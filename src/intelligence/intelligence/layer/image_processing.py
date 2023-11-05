import torch
import torch.nn as nn


class ImageEncoder(nn.Module):
    def __init__(self, feat_dim):
        super(ImageEncoder, self).__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=3, stride=2, padding=1),
            nn.LayerNorm([64, 64, 64]),
            nn.ReLU(True),
            nn.Conv2d(64, 32, 3, 2, 1),
            nn.LayerNorm([32, 32, 32]),
            nn.ReLU(True),
            nn.Conv2d(32, 16, 3, 2, 1),
            nn.LayerNorm([16, 16, 16]),
            nn.ReLU(True),
            nn.Conv2d(16, 12, 3, 2, 1),
            nn.LayerNorm([12, 8, 8]),
            nn.ReLU(True),
            nn.Conv2d(12, 8, 3, 2, 1),
            nn.LayerNorm([8, 4, 4]),
            nn.ReLU(True),
            nn.Flatten(),
            nn.Linear(8 * 4 * 4, 50),
            nn.LayerNorm([50]),
            nn.ReLU(True),
            nn.Linear(50, feat_dim),
            nn.LayerNorm([feat_dim]),
            nn.ReLU(True),
        )

    def forward(self, image):
        return self.encoder(image)


class ImageEncoder256(nn.Module):
    def __init__(self, feat_dim):
        super(ImageEncoder256, self).__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 8, 3, 2, 1),
            nn.LayerNorm([8, 128, 128]),
            nn.ReLU(True),
            nn.Conv2d(8, 16, 3, 2, 1),
            nn.LayerNorm([16, 64, 64]),
            nn.ReLU(True),
            nn.Conv2d(16, 32, 3, 2, 1),
            nn.LayerNorm([32, 32, 32]),
            nn.ReLU(True),
            nn.Conv2d(32, 64, 3, 2, 1),
            nn.LayerNorm([64, 16, 16]),
            nn.ReLU(True),
            nn.Conv2d(64, 128, 3, 2, 1),
            nn.LayerNorm([128, 8, 8]),
            nn.ReLU(True),
            nn.Conv2d(128, 128, 3, 2, 1),
            nn.LayerNorm([128, 4, 4]),
            nn.ReLU(True),
            nn.Flatten(),
            nn.Linear(4 * 4 * 128, 128),
            nn.LayerNorm([128]),
            nn.ReLU(True),
            nn.Linear(128, feat_dim),
            nn.LayerNorm([feat_dim]),
            nn.ReLU(True),
        )

    def forward(self, image):
        return self.encoder(image)


class ImageDecoder(nn.Module):
    def __init__(self, feat_dim):
        super(ImageDecoder, self).__init__()
        self.decoder = nn.Sequential(
            nn.Linear(feat_dim, 8 * 4 * 4),
            nn.LayerNorm([8 * 4 * 4]),
            nn.ReLU(True),
            nn.Unflatten(1, (8, 4, 4)),
            nn.ConvTranspose2d(8, 12, 3, 2, padding=1, output_padding=1),
            nn.LayerNorm([12, 8, 8]),
            nn.ReLU(True),
            nn.ConvTranspose2d(12, 16, 3, 2, 1, 1),
            nn.LayerNorm([16, 16, 16]),
            nn.ReLU(True),
            nn.ConvTranspose2d(16, 32, 3, 2, 1, 1),
            nn.LayerNorm([32, 32, 32]),
            nn.ReLU(True),
            nn.ConvTranspose2d(32, 64, 3, 2, 1, 1),
            nn.LayerNorm([64, 64, 64]),
            nn.ReLU(True),
            nn.ConvTranspose2d(64, 3, 3, 2, 1, 1),
            nn.ReLU(True),
        )

    def forward(self, feat):
        return self.decoder(feat)


class ImageDecoder256(nn.Module):
    def __init__(self, feat_dim):
        super(ImageDecoder256, self).__init__()
        self.decoder = nn.Sequential(
            nn.Linear(feat_dim, 4 * 4 * 128),
            nn.LayerNorm([4 * 4 * 128]),
            nn.ReLU(True),
            nn.Unflatten(1, (128, 4, 4)),
            nn.ConvTranspose2d(128, 128, 3, 2, 1, 1),
            nn.LayerNorm([128, 8, 8]),
            nn.ReLU(True),
            nn.ConvTranspose2d(128, 64, 3, 2, 1, 1),
            nn.LayerNorm([64, 16, 16]),
            nn.ReLU(True),
            nn.ConvTranspose2d(64, 32, 3, 2, 1, 1),
            nn.LayerNorm([32, 32, 32]),
            nn.ReLU(True),
            nn.ConvTranspose2d(32, 16, 3, 2, 1, 1),
            nn.LayerNorm([16, 64, 64]),
            nn.ReLU(True),
            nn.ConvTranspose2d(16, 8, 3, 2, 1, 1),
            nn.LayerNorm([8, 128, 128]),
            nn.ReLU(True),
            nn.ConvTranspose2d(8, 3, 3, 2, 1, 1),
            nn.ReLU(True),
        )

    def forward(self, feat):
        return self.decoder(feat)
