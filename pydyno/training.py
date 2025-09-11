import os

import torch

#from torch import Module, nn

from torch.utils.data import DataLoader, Dataset, random_split

from pathlib import Path

#train_dataloader = DataLoader(training_data, batch_size=64, shuffle=True)
#test_dataloader = DataLoader(test_data, batch_size=64, shuffle=True)


class DynoHDF5Dataset(Dataset):

    def __init__(self: DynoHDF5Dataset, path: Path):
        # store the inputs and outputs
        #self._data = ...

        pass

    # number of rows in the dataset
    def __len__(self):

        return len(self.X)

    # get a row at an index
    def __getitem__(self, idx):
        return [self.X[idx], self.y[idx]]


dataset = DynoHDF5Dataset("./dataset.h5")

training_data, testing_data = random_split(dataset, [[...], [...]])

# create a data loader for train and test sets
training_dataloader = DataLoader(training_data, batch_size=32, shuffle=True)
testing_dataloader = DataLoader(testing_data, batch_size=1024, shuffle=False)

# train the model
for index, (inputs, targets) in enumerate(training_dataloader):
    pass


# model definition
class MLP(Module):
    # define model elements
    def __init__(self, n_inputs):
        super(MLP, self).__init__()
        self.layer = Linear(n_inputs, 1)
        self.activation = Sigmoid()

    # forward propagate input
    def forward(self, X):
        X = self.layer(X)
        X = self.activation(X)
        return X


device = ("cuda" if torch.cuda.is_available() else
          "mps" if torch.backends.mps.is_available() else "cpu")
print(f"Using {device} device")


class NeuralNetwork(nn.Module):

    def __init__(self):

        super().__init__()

        self.flatten = nn.Flatten()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(28 * 28, 512),
            nn.ReLU(),
            nn.Linear(512, 512),
            nn.ReLU(),
            nn.Linear(512, 10),
        )

    def forward(self, x):
        x = self.flatten(x)
        logits = self.linear_relu_stack(x)
        return logits


model = NeuralNetwork().to(device)
print(model)

X = torch.rand(1, 28, 28, device=device)
logits = model(X)
pred_probab = nn.Softmax(dim=1)(logits)
y_pred = pred_probab.argmax(1)
print(f"Predicted class: {y_pred}")

from torch.nn import BCELoss
from torch.optim import SGD


from sklearn.preprocessing import LabelEncoder

# dataset definition
class CSVDataset(Dataset):
    # load the dataset
    def __init__(self, path):
        # load the csv file as a dataframe
        df = read_csv(path, header=None)

        # store the inputs and outputs
        self.X = df.values[:, :-1]
        self.y = df.values[:, -1]

        # ensure input data is floats
        self.X = self.X.astype('float32')

        # label encode target and ensure the values are floats
        self.y = LabelEncoder().fit_transform(self.y)
        self.y = self.y.astype('float32')
        self.y = self.y.reshape((len(self.y), 1))

    # number of rows in the dataset
    def __len__(self):

        return len(self.X)

    # get a row at an index
    def __getitem__(self, idx):

        return [self.X[idx], self.y[idx]]

    # get indexes for train and test rows
    def get_splits(self, n_test=0.33):
        # determine sizes
        test_size = round(n_test * len(self.X))
        train_size = len(self.X) - test_size
        # calculate the split
        return random_split(self, [train_size, test_size])

# train the model
def train_model(train_dl, model, epochs: int = 100):
    # define the optimization
    criterion = BCELoss()
    optimizer = SGD(model.parameters(), lr=0.01, momentum=0.9)

    # enumerate epochs
    for epoch in range(epochs):
        # enumerate mini batches
        for index, (inputs, targets) in enumerate(train_dl):
            # clear the gradients
            optimizer.zero_grad()
            # compute the model output
            yhat = model(inputs)
            # calculate loss
            loss = criterion(yhat, targets)
            # credit assignment
            loss.backward()
            # update model weights
            optimizer.step()
