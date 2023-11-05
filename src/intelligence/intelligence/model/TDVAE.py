import torch
import torch.nn as nn
import torch.nn.functional as F

from layer.image_processing import ImageEncoder, ImageDecoder
from layer.MTRNN import MTRNNCell


class DBlock(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(DBlock, self).__init__()

        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size

        self.fc1 = nn.Linear(input_size, hidden_size)
        self.fc2 = nn.Linear(input_size, hidden_size)
        self.fc_mu = nn.Linear(hidden_size, output_size)
        self.fc_logsigma = nn.Linear(hidden_size, output_size)

    def forward(self, input):
        t = torch.tanh(self.fc1(input))
        t = t * torch.sigmoid(self.fc2(input))
        mu = self.fc_mu(t)
        logsigma = self.fc_logsigma(t)
        return mu, logsigma


class TDVAE(nn.Module):
    def __init__(self,
                 image_feat_dim,
                 joint_dim,
                 rnn_hidden_dims,
                 rnn_taus,
                 b_dim,
                 z_dim,
                 ):
        super(TDVAE, self).__init__()

        self.image_feat_dim = image_feat_dim
        self.joint_dim = joint_dim
        self.rnn_hidden_dims = rnn_hidden_dims
        self.rnn_taus = rnn_taus

        self.b_dim = b_dim
        self.z_dim = z_dim

        self.image_encoder = ImageEncoder(self.image_feat_dim)

        self.rnn = MTRNNCell(input_dim=self.image_feat_dim + joint_dim,
                             io_hidden_dim=self.rnn_hidden_dims[0],
                             fast_hidden_dim=self.rnn_hidden_dims[1],
                             slow_hidden_dim=self.rnn_hidden_dims[2],
                             io_tau=self.b_dim,
                             fast_tau=self.b_dim,
                             slow_tau=self.b_dim)

        self.l2_b2z = DBlock(b_dim, 64, z_dim)
        self.l1_b2z = DBlock(b_dim + z_dim, 64, self.z_dim)

        self.l2_infer_z = DBlock(b_dim + 2 * z_dim, 64, z_dim)
        self.l1_infer_z = DBlock(b_dim + 2 * z_dim + z_dim, 64, z_dim)

        self.l2_trans_z = DBlock(2 * z_dim, 64, z_dim)
        self.l1_trans_z = DBlock(2 * z_dim + z_dim, 64, z_dim)

        self.image_decoder = ImageDecoder(2 * z_dim)
        self.joint_decoder = nn.Sequential(nn.Linear(2 * z_dim, joint_dim),
                                           nn.ReLU(True))

    def forward(self, image, joint, state=None):
        img_feat = self.image_encoder(image)
