import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
topic = "/ov_msckf/odomimu"
reader = SequentialReader()
reader.open(StorageOptions(uri=bag, storage_id=""),
            ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
msg_cls = get_message(types[topic])

t0 = None
last_print = -999
while reader.has_next():
    name, data, t = reader.read_next()
    if name != topic:
        continue
    msg = deserialize_message(data, msg_cls)
    if t0 is None:
        t0 = t
    rel = (t - t0) / 1e9
    if rel - last_print >= 1.0:   # print ~once per second
        p = msg.pose.pose.position
        print(f"t={rel:6.1f}s  x={p.x:+.3f}  y={p.y:+.3f}  z={p.z:+.3f}")
        last_print = rel
