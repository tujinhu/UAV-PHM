import torch
import torch.nn as nn

class ConvBlock1D(nn.Module):
    def __init__(self, in_ch, out_ch, kernel_size=3, padding=None, stride=1, dropout=0.0):
        super().__init__()
        if padding is None:
            padding = (kernel_size - 1) // 2  # preserve length by default
        self.conv = nn.Conv1d(in_ch, out_ch, kernel_size=kernel_size, stride=stride, padding=padding)
        self.bn = nn.BatchNorm1d(out_ch)
        self.act = nn.ReLU(inplace=True)
        self.dropout = nn.Dropout(dropout) if dropout > 0.0 else nn.Identity()

    def forward(self, x):
        # x: (B, C, T)
        x = self.conv(x)
        x = self.bn(x)
        x = self.act(x)
        x = self.dropout(x)
        return x  # (B, out_ch, T')

class ConvBiLSTMEncoder(nn.Module):
    def __init__(self,
                 in_channels=6,
                 conv_filters=[48, 64, 128],
                 conv_kernels=[5, 5, 3],
                 conv_dropouts=[0.1, 0.1, 0.1],
                 lstm_hidden=128,
                 lstm_layers=2,
                 bidirectional=True,
                 dropout_lstm=0.2):
        super().__init__()

        assert len(conv_filters) == len(conv_kernels) == len(conv_dropouts)
        self.conv_blocks = nn.ModuleList()
        prev_ch = in_channels
        for out_ch, k, d in zip(conv_filters, conv_kernels, conv_dropouts):
            self.conv_blocks.append(ConvBlock1D(prev_ch, out_ch, kernel_size=k, padding=(k-1)//2, dropout=d))
            prev_ch = out_ch

        # Bi-LSTM expects input (T, B, feat). We'll feed conv features after permute.
        self.lstm_input_size = prev_ch  # because conv keeps time dimension as last dim
        self.lstm = nn.LSTM(input_size=self.lstm_input_size,
                            hidden_size=lstm_hidden,
                            num_layers=lstm_layers,
                            batch_first=True,  # we will use (B, T, feat)
                            bidirectional=bidirectional,
                            dropout=dropout_lstm if lstm_layers > 1 else 0.0)
        self.bidirectional = bidirectional
        self.lstm_hidden = lstm_hidden
        self.lstm_layers = lstm_layers

    def forward(self, x, return_sequence=True):
        # permute to (B, C, T) for conv1d
        x = x.permute(0, 2, 1)  # (B, C, T)
        for conv in self.conv_blocks:
            x = conv(x)  # maintain (B, C', T)

        # prepare for LSTM: (B, T, feat)
        x = x.permute(0, 2, 1)  # (B, T, feat)
        # Bi-LSTM
        lstm_out, (h_n, c_n) = self.lstm(x)  # lstm_out: (B, T, hidden * num_dir)
        # h_n: (num_layers * num_dirs, B, hidden)
        if self.bidirectional:
            # take last layer's forward and backward states and concat
            # index for last layer:
            idx_fwd = (self.lstm_layers - 1) * 2
            idx_bwd = idx_fwd + 1
            h_fwd = h_n[idx_fwd]  # (B, hidden)
            h_bwd = h_n[idx_bwd]  # (B, hidden)
            embedding = torch.cat([h_fwd, h_bwd], dim=-1)  # (B, hidden*2)
        else:
            embedding = h_n[-1]  # (B, hidden)

        if return_sequence:
            return embedding, lstm_out  # (B, emb_dim), (B, T, hidden*dirs)
        else:
            return embedding  # (B, emb_dim)

class FaultClassifier(nn.Module):
    def __init__(self,
                 encoder: ConvBiLSTMEncoder,
                 num_classes=6,
                 fc_hidden=[128],
                 dropout_fc=0.3):
        super().__init__()
        self.encoder = encoder

        # additional Bi-LSTM on top of encoder sequence output (paper: unrolled through Bi-LSTM)
        enc_out_dim = encoder.lstm_hidden * (2 if encoder.bidirectional else 1)
    
        # FC head
        fc_input_dim = enc_out_dim
        fc_layers = []
        prev = fc_input_dim
        for h in fc_hidden:
            fc_layers.append(nn.Linear(prev, h))
            fc_layers.append(nn.ReLU(inplace=True))
            fc_layers.append(nn.Dropout(dropout_fc))
            prev = h
        fc_layers.append(nn.Linear(prev, num_classes))
        self.fc = nn.Sequential(*fc_layers)

    def forward(self, x):
        embedding, seq = self.encoder(x, return_sequence=True)
        logits = self.fc(embedding)  # (B, num_classes)
        return logits

    