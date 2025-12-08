import torch
import torch.nn as nn
import torch.nn.functional as F
import math

EPS = 1e-8
########### MDN helper funcs ###########
def parse_mdn_outputs(z, K, d):
    """
    z: [B, T, out_dim] where out_dim = K*(1 + 2*d)
    returns:
      pi:   [B, T, K]
      mu:   [B, T, K, d]
      sigma:[B, T, K, d]   (std, positive)
    """
    B, T, out_dim = z.shape
    expected = K * (1 + 2 * d)
    assert out_dim == expected, f"out_dim mismatch: got {out_dim}, expected {expected}"
    # split
    z = z.view(B, T, -1)
    pi_logits = z[..., :K]                      # [B,T,K]
    mu_flat = z[..., K: K + K * d]             # [B,T,K*d]
    sigma_flat = z[..., K + K * d:]            # [B,T,K*d]
    mu = mu_flat.view(B, T, K, d)
    sigma = sigma_flat.view(B, T, K, d)
    # mixture weights with numerical stability
    pi = F.softmax(pi_logits, dim=-1)          # [B,T,K]
    # ensure positivity & numerical stability for sigma (model outputs log-sigma or raw)
    # we treat sigma_flat as unconstrained -> apply softplus or exp
    sigma = F.softplus(sigma) + 1e-6           # [B,T,K,d], softplus is stable
    return pi, mu, sigma

def mdn_nll(pi, mu, sigma, target, reduce_mean=True):
    """
    Negative log-likelihood of target under diagonal Gaussian mixture.
    pi: [B,T,K], mu: [B,T,K,d], sigma: [B,T,K,d], target: [B,T,d]
    returns: scalar loss (mean over batch & time) if reduce_mean True, else per-sample [B,T]
    """
    B, T, K = pi.shape
    d = target.shape[-1]
    tgt = target.unsqueeze(2)                   # [B,T,1,d]
    # compute exponent term: -0.5 * sum_d ((x - mu)^2 / sigma^2)
    var_term = ((tgt - mu) / (sigma + EPS)) ** 2   # [B,T,K,d]
    exp_term = -0.5 * torch.sum(var_term, dim=-1)  # [B,T,K]
    # log sigma term: - sum_d log(sigma)
    log_sigma = - torch.sum(torch.log(sigma + EPS), dim=-1)  # [B,T,K]
    const = -0.5 * d * math.log(2 * math.pi)                 # scalar
    log_prob_components = exp_term + log_sigma + const       # [B,T,K]
    log_pi = torch.log(pi + EPS)                             # [B,T,K]
    log_weighted = log_pi + log_prob_components              # [B,T,K]
    # log-sum-exp over mixture components => log p(x)
    log_prob = torch.logsumexp(log_weighted, dim=-1)         # [B,T]
    nll = -log_prob                                           # [B,T]
    if reduce_mean:
        return torch.mean(nll)
    else:
        return nll

def mdn_sample_step(pi, mu, sigma):
    """
    Sample one step from mixture.
    pi: [B, K], mu: [B, K, d], sigma: [B, K, d]
    returns sampled tensor [B, d]
    """
    B, K, d = mu.shape
    # sample component index for each batch
    comp = torch.multinomial(pi, num_samples=1).squeeze(-1)  # [B]
    idx = comp.view(B, 1, 1).expand(-1, 1, d)               # [B,1,d]
    mu_sel = torch.gather(mu, dim=1, index=idx).squeeze(1)  # [B,d]
    sigma_sel = torch.gather(sigma, dim=1, index=idx).squeeze(1)  # [B,d]
    eps = torch.randn_like(mu_sel)
    return mu_sel + sigma_sel * eps

def mdn_conditional_mean(pi, mu):
    """
    Compute conditional expectation E[x] = sum_k pi_k * mu_k
    pi: [B,T,K], mu: [B,T,K,d] -> returns [B,T,d]
    """
    # expand & sum
    # pi[..., None] * mu -> [B,T,K,d]
    mean = torch.sum(pi.unsqueeze(-1) * mu, dim=2)  # [B,T,d]
    return mean

def mdn_mixture_covariance(pi, mu, sigma):
    """
    Compute mixture covariance per time-step:
      Cov = sum_k pi_k (Sigma_k + mu_k mu_k^T) - mu_bar mu_bar^T
    pi: [B,T,K], mu: [B,T,K,d], sigma: [B,T,K,d] (sigma is std)
    returns: cov_diag [B,T,d] (diagonal covariance across dims)
    Note: we return diagonal cov for simplicity (matches diagonal Gaussian assumption).
    """
    # convert sigma (std) to var
    var = sigma ** 2  # [B,T,K,d]
    mu_bar = mdn_conditional_mean(pi, mu)  # [B,T,d]
    # compute E[xx^T] diagonal = sum_k pi_k (var_k + mu_k^2)
    exx_diag = torch.sum(pi.unsqueeze(-1) * (var + mu ** 2), dim=2)  # [B,T,d]
    cov_diag = exx_diag - mu_bar ** 2  # [B,T,d]
    return cov_diag  # diagonal of covariance

# ------------------------------
# Generator: LSTM + FC -> MDN params (supports conditional vector c_t)
# ------------------------------
class ErrGenerator(nn.Module):
    def __init__(self, out_dim=3, cond_dim=0, hidden_size=256, num_layers=2, fc_dim=128, num_mixtures=24, act='relu'):
        """
        out_dim: d (residual dimensionality)
        cond_dim: dimensionality of conditional vector c_t (can be 0)
        The input to LSTM at time t is [E_{t-1}, c_t] as specified.
        """
        super().__init__()
        self.d = out_dim
        self.cond_dim = cond_dim
        self.inp_dim = out_dim + cond_dim
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.num_mixtures = num_mixtures

        self.lstm = nn.LSTM(input_size=self.inp_dim, hidden_size=hidden_size,
                            num_layers=num_layers, batch_first=True)
        self.fc1 = nn.Linear(hidden_size, fc_dim)
        self.fc2 = nn.Linear(fc_dim, num_mixtures * (1 + 2 * self.d))  # K*(1 + 2*d)
        if act == 'sigmoid':
            self.act = torch.sigmoid
        elif act == 'tanh':
            self.act = torch.tanh
        elif act == 'relu':
            self.act = F.leaky_relu  
        else:
            self.act = lambda x: x  # identity

    def forward(self, prev_E, cond=None, hidden=None):
        """
        prev_E: tensor [B, T, d]  (these are inputs E_{t-1} aligned to outputs for next step)
        cond:   optional tensor [B, T, cond_dim] (if cond_dim>0)
        returns: pi [B,T,K], mu [B,T,K,d], sigma [B,T,K,d]
        """
        B, T, d_ = prev_E.shape
        assert d_ == self.d
        if self.cond_dim > 0:
            assert cond is not None
            assert cond.shape[0] == B and cond.shape[1] == T
            inp = torch.cat([prev_E, cond], dim=-1)  # [B,T,d+cond]
        else:
            inp = prev_E  # [B,T,d]
        out_seq, hidden = self.lstm(inp, hidden)     # out_seq: [B, T, hidden]
        z = self.act(self.fc1(out_seq))              # [B,T,fc_dim]
        z = self.fc2(z)                              # [B,T, out_dim]
        pi, mu, sigma = parse_mdn_outputs(z, self.num_mixtures, self.d)
        return pi, mu, sigma, hidden
    
    @torch.no_grad()
    def sample(self, seq_len, batch_size=1, device='cpu', start_input=None, cond_seq=None):
        """
        Autoregressive sampling:
        start_input: [B, d] or None -> used as E_0 input for first step
        cond_seq: optional [B, seq_len, cond_dim]
        returns samples [B, seq_len, d]
        """
        d = self.d
        if start_input is None:
            prev = torch.zeros(batch_size, 1, d, device=device)
        else:
            prev = start_input.view(batch_size, 1, d).to(device)
        # initialize hidden states
        h0 = torch.zeros(self.num_layers, batch_size, self.hidden_size, device=device)
        c0 = torch.zeros(self.num_layers, batch_size, self.hidden_size, device=device)
        hidden = (h0, c0)
        samples = []
        for t in range(seq_len):
            cond_t = None
            if self.cond_dim > 0 and cond_seq is not None:
                cond_t = cond_seq[:, t:t+1, :]  # [B,1,cond_dim]
                inp = torch.cat([prev, cond_t], dim=-1)
            else:
                inp = prev
            out_seq, hidden = self.lstm(inp, hidden)   # out_seq [B,1,hidden]
            z = self.act(self.fc1(out_seq))            # [B,1,fc_dim]
            z = self.fc2(z)                            # [B,1,out_dim]
            pi, mu, sigma = parse_mdn_outputs(z.squeeze(1).unsqueeze(1), self.num_mixtures, d)
            # pi: [B,1,K] -> squeeze to [B,K]
            pi_step = pi.squeeze(1)    # [B,K]
            mu_step = mu.squeeze(1)    # [B,K,d]
            sigma_step = sigma.squeeze(1)  # [B,K,d]
            next_x = mdn_sample_step(pi_step, mu_step, sigma_step)  # [B,d]
            samples.append(next_x.unsqueeze(1))
            prev = next_x.unsqueeze(1)
        return torch.cat(samples, dim=1)  # [B, seq_len, d]

# ------------------------------
# Discriminators (WGAN critics)
# - Time-domain critic: LSTM -> FC -> scalar
# - Frequency-domain critic: compute rFFT along time, feed real+imag sequence to LSTM -> scalar
# ------------------------------
class TimeCritic(nn.Module):
    def __init__(self, input_dim=3, hidden_size=128, num_layers=1, dropout=0.1):
        super().__init__()
        self.lstm = nn.LSTM(input_size=input_dim, hidden_size=hidden_size,
                            num_layers=num_layers, batch_first=True, bidirectional=False)

        self.fc1 = nn.Linear(hidden_size, hidden_size // 2)
        self.fc2 = nn.Linear(hidden_size // 2, 1)  # 最终输出1个logit
        self.dropout = nn.Dropout(dropout)
        self.act = F.leaky_relu 

    def forward(self, x):
        """x: [B, T, d] -> returns scores [B]"""
        out_seq, _ = self.lstm(x)   # [B,T,hidden]
        last = out_seq[:, -1, :]    # 取最后一个时间步的特征
        z = self.act(self.fc1(last))
        z = self.dropout(z)
        score = self.fc2(z).squeeze(-1)  # [B]（logits）
        return score

class FreqCritic(nn.Module):
    def __init__(self, input_dim=3, hidden_size=128, num_layers=1, use_log_mag=True, dropout=0.1):
        super().__init__()
        self.use_log_mag = use_log_mag
        # 频率特征维度：use_log_mag时为d（log幅度），否则为2*d（实部+虚部）
        self.feat_dim = input_dim if use_log_mag else 2 * input_dim
        self.lstm = nn.LSTM(input_size=self.feat_dim, hidden_size=hidden_size,
                            num_layers=num_layers, batch_first=True)
        # 新增：稳定训练的组件
        self.ln = nn.LayerNorm(self.feat_dim)
        self.fc1 = nn.Linear(hidden_size, hidden_size // 2)
        self.fc2 = nn.Linear(hidden_size // 2, 1)
        self.dropout = nn.Dropout(dropout)
        self.act = F.leaky_relu

    def forward(self, x):
        """
        x: [B, T, d]（时域序列）
        正确逻辑：
        1. 时域转频域（rfft）-> [B, T_freq, d]（复数）
        2. 特征提取：use_log_mag=True → log幅度（[B,T_freq,d]）；否则 → 实部+虚部（[B,T_freq,2d]）
        3. 归一化+LSTM+FC → logits
        """
        # 1. 时域转频域（沿时间轴dim=1）
        Xf = torch.fft.rfft(x, dim=1)  # [B, T_freq, d]（复数）
        real = Xf.real
        imag = Xf.imag

        # 2. 提取频率特征（修复核心错误）
        if self.use_log_mag:
            # 计算幅度：sqrt(real² + imag²)，加EPS避免0
            mag = torch.sqrt(real ** 2 + imag ** 2 + EPS)
            feat = torch.log(mag + EPS)  # [B, T_freq, d]（log幅度，数值更稳定）
        else:
            feat = torch.cat([real, imag], dim=-1)  # [B, T_freq, 2d]（实部+虚部）

        # 3. 归一化（关键：稳定数值范围，避免梯度爆炸）
        feat = self.ln(feat)  # [B, T_freq, feat_dim]

        # 4. LSTM+FC特征提取
        out_seq, _ = self.lstm(feat)  # [B, T_freq, hidden]
        last = out_seq[:, -1, :]      # 取最后一个频率步的特征
        z = self.act(self.fc1(last))
        z = self.dropout(z)
        score = self.fc2(z).squeeze(-1)  # [B]（logits）

        return score
