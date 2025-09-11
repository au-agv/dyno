from __future__ import annotations

import copy
import logging

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim

from sklearn.model_selection import train_test_split
from sklearn.datasets import fetch_california_housing
from sklearn.preprocessing import StandardScaler

from pathlib import Path

import h5py
import tqdm


class DynoHDF5Dataset:

    def __init__(self, file_path: Path):

        self._load_file(file_path)
        self._load_data()

    def _load_file(self, file_path: Path):

        self._data: h5py.File = h5py.File(self._file_path)

    def _load_data(self):

        self._rms: float = self._data["rms"]
        self._roughness: float = self._data["roughness"]
        self._waviness: float = self._data["waviness"]
        self._seat_acceleration: float = self._data["seat_acceleration"]


class Trainer:

    def __init__(
        self,
        input_data,
        output_data,
        learning_rate: float = 0.0001,
    ) -> None:

        self._input_data = input_data
        self._output_data = output_data

        self._scaler = StandardScaler()

        (self._input_training_data, self._output_training_data), (
            self._input_test_data,
            self._output_test_data) = self._split_data()

        self.normalize_input_data()
        self.get_tensors()

        # Define the neural network module.
        self._model = nn.Sequential(nn.Linear(8, 24), nn.ReLU(),
                                    nn.Linear(24, 12), nn.ReLU(),
                                    nn.Linear(12, 6), nn.ReLU(),
                                    nn.Linear(6, 1))

        # Define the loss function.
        self._loss_function = nn.MSELoss()

        # Define the optimizer.
        self._optimizer = optim.Adam(self._model.parameters(),
                                     lr=learning_rate)

    def _split_data(self, training_partition: float = 0.8):

        # Split the training dataset in a training partition and a testing partition.
        input_training_data, input_test_data, output_training_data, output_test_data = train_test_split(
            self._input_data,
            self._output_data,
            train_size=training_partition,
            shuffle=True)

        return (input_training_data, output_training_data), (input_test_data,
                                                             output_test_data)

    def normalize_input_data(self):

        # Standardize the dataset by removing the mean scaling to unit variance.
        self._scaler.fit(self._input_training_data)
        self._scaled_training_data = self._scaler.transform(
            self._input_training_data)
        self._scaled_test_data = self._scaler.transform(self._input_test_data)

    def get_tensors(self):

        # Conver the input data to two-dimensional Torch tensors.

        self._X_train = torch.tensor(self._scaled_training_data,
                                     dtype=torch.float32)
        self._y_train = torch.tensor(self._output_training_data,
                                     dtype=torch.float32).reshape(-1, 1)
        self._X_test = torch.tensor(self._scaled_test_data,
                                    dtype=torch.float32)
        self._y_test = torch.tensor(self._output_test_data,
                                    dtype=torch.float32).reshape(-1, 1)

    def train(self, epochs: int = 100, batch_size: int = 10):

        batch_start = torch.arange(0, len(self._X_train), batch_size)

        # Hold the best model
        best_mse = np.inf  # init to infinity
        best_weights = None
        history = []

        for epoch in range(epochs):

            # Explicitly set the module to training mode.
            self._model.train()

            with tqdm.tqdm(batch_start,
                           unit="batch",
                           mininterval=0,
                           disable=False) as bar:
                bar.set_description(f"Epoch {epoch}")
                for start in bar:
                    # Collect a batch from the training data.
                    X_batch = self._X_train[start:start + batch_size]
                    y_batch = self._y_train[start:start + batch_size]

                    # Perform a forward pass of the model and evaluate the current loss.
                    y_pred = self._model(X_batch)
                    loss = self._loss_function(y_pred, y_batch)

                    # Perform a backward pass of the model.
                    self._optimizer.zero_grad()
                    loss.backward()

                    # Update the model weights.
                    self._optimizer.step()

                    # Show progress for this training epoch.
                    bar.set_postfix(mse=float(loss))

                    # evaluate accuracy at end of each epoch
                    self._model.eval()
                    y_pred = self._model(self._X_test)
                    mse = self._loss_function(y_pred, self._y_test)
                    mse = float(mse)
                    history.append(mse)
                    if mse < best_mse:
                        best_mse = mse
                        best_weights = copy.deepcopy(self._model.state_dict())

        self._export_weights(best_weights)

    def _export_weights(self: Trainer, weights: Dict[str, any]) -> None:
        # restore model and return best accuracy
        self._model.load_state_dict(best_weights)
        print("MSE: %.2f" % best_mse)
        print("RMSE: %.2f" % np.sqrt(best_mse))
        plt.plot(history)
        plt.show()

        # Print model's state_dict
        print("Model's state_dict:")
        for param_tensor in self._model.state_dict():
            print(param_tensor, "\t",
                  self._model.state_dict()[param_tensor].size())

        # Print optimizer's state_dict
        print("Optimizer's state_dict:")
        for var_name in self._optimizer.state_dict():
            print(var_name, "\t", self._optimizer.state_dict()[var_name])


def main():

    # Import the training dataset.
    data = fetch_california_housing()
    print(data)
    x, y = data.data, data.target

    trainer = Trainer(x, y)
    trainer.train()

    logger = logging.getLogger(__name__)
    """
    logger.info("Saving model weights ...")
    torch.save(model.state_dict(), "./weights.pt")

    logger.info("Exporting the trained model to TorchScript ...")
    model_scripted = torch.jit.script(model)  # Export to TorchScript
    model_scripted.save('weights_torchscript.pt')  # Save

    model.eval()
    with torch.no_grad():
        # Test out inference with 5 samples
        for i in range(5):
            X_sample = X_test_raw[i:i + 1]
            X_sample = scaler.transform(X_sample)
            X_sample = torch.tensor(X_sample, dtype=torch.float32)
            y_pred = model(X_sample)
            print(
                f"{X_test_raw[i]} -> {y_pred[0].numpy()} (expected {y_test[i].numpy()})"
            )
    """


if __name__ == "__main__":

    main()
