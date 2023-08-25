from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
from .viz import UsageViz

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('path')
    parser.add_argument('-p', '--progressive', action='store_true')
    args = parser.parse_args()
    format = 'sqlite3'
    serialization_format = 'cdr'
    storage_options = StorageOptions(str(args.path), format)
    converter_options = ConverterOptions(serialization_format, serialization_format)

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    storage_filter = StorageFilter(topics=['/cpu_usage'])
    reader.set_filter(storage_filter)
    topic_types = reader.get_all_topics_and_types()
    type_map = {tmeta.name: tmeta.type for tmeta in topic_types}

    uv = UsageViz(args.progressive)

    while reader.has_next():
        (topic, rawdata, timestamp) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(rawdata, msg_type)
        ts = timestamp / 1e9
        uv.update(ts, msg)

    uv.show()


if __name__ == '__main__':
    main()
