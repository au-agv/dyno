from numpy import array, vstack

from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from tables import open_file

import matplotlib.pyplot as plt


class BagPostprocessor:

    def __init__(self, path: str):

        self._path = Path(path)
        self._typestore = get_typestore(Stores.ROS2_HUMBLE)
        self._file = open_file(f'{self._path.with_suffix(".h5")}', mode="w")
        self._file.create_group("/", "data")
        self._load()

    def _load(self):

        with AnyReader([self._path],
                       default_typestore=self._typestore) as reader:
            connections = [
                x for x in reader.connections
                if x.msgtype == 'std_msgs/msg/Float64'
                and x.topic.endswith('throttle')
            ]

            times = []
            values = []

            for connection, timestamp, rawdata in reader.messages(
                    connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)

                times.append(timestamp / 1.0e9)
                values.append(msg.data)

            self._to_entry(
                self._to_array(times) - min(times),
                self._to_array(values), "myentry")

    def _to_array(self, values):

        return array(values)

    def _to_entry(self, times, value, entry):

        table = self._file.create_array("/data", entry, vstack([times, value]))

    def close(self):

        self._file.close()


def main():

    PATH = Path(
        '/home/dsirangelo/Repositories/github.com/aarhus-robotics/dyno/examples/reliability/autonomous_navigation/run.1/bag'
    )

    postprocessor = BagPostprocessor(PATH.as_posix())
    postprocessor.close()

    file = open_file(f'{PATH.with_suffix(".h5")}', mode="r")
    plt.subplots(1, 1)
    myarray = array(file.root["/data/myentry"])
    print(myarray.shape)
    plt.plot(myarray[0, :], myarray[1, :])
    file.close()
    plt.show()


if __name__ == '__main__':
    main()
