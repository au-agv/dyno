import tables
import matplotlib.pyplot as plt


def plot_trajectory(x, y, axes):

    axes.set_aspect("equal")
    axes.plot(x, y)
    figure.tight_layout()


import pathlib

files = list(pathlib.Path.cwd().glob("*.h5"))
print(files)
figure, axes = plt.subplots(1, 1)
for file in files:
    with tables.open_file(file.as_posix(), mode="r") as h5file:
        print("OPENING FILE %s", file.as_posix())
        table = h5file.root.data.pose.position
        # 'entry_name' is name of your HDF5 file

        # read into a NumPy arra

        plot_trajectory(
            h5file.root.data.pose.position[:, 0],
            h5file.root.data.pose.position[:, 1],
            axes,
        )
plt.show()
