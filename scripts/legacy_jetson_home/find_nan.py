import sys
import numpy as np
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

i = 0
first_nan = None
nan_count = 0
last_good = None
while reader.has_next():
    name, data, t = reader.read_next()
    if name != topic:
        continue
    msg = deserialize_message(data, msg_cls)
    p = msg.pose.pose.position
    xyz = (p.x, p.y, p.z)
    if any(np.isnan(v) for v in xyz):
        nan_count += 1
        if first_nan is None:
            first_nan = (i, t, xyz, last_good)
    else:
        last_good = (i, t, xyz)
    i += 1

print(f"total poses: {i}")
print(f"nan poses: {nan_count}")
if first_nan:
    idx, t, xyz, prev = first_nan
    print(f"first NaN at index {idx}, t={t/1e9:.2f}s, value={xyz}")
    print(f"last good pose before it: {prev}")
else:
    print("no NaN found (check the flight_analysis.py math instead)")
