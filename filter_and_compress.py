from email.mime import base
import rosbag
import argparse
import glob
import os
to_keep = ['/livox_raw', '/velodyne_rot', '/velodyne_static_raw', '/imu/data_hwts']
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input_bag_dir", help="input bag ")
    parser.add_argument("-o", "--output_bag_dir", help="output bag ")
    
    # walk for bags
    args = parser.parse_args()
    print("Input dir %s"%args.input_bag_dir )
    input_bags = glob.glob(args.input_bag_dir+"/"+"*.bag")
    output_dir = args.output_bag_dir

    for f in input_bags:
        base_name = os.path.basename(f)[:-4]
        print("processing %s " % base_name)
        inbagfile = rosbag.Bag(f)
        outbagfile = rosbag.Bag(output_dir+"/"+base_name+"_filter.bag", 'w', compression=rosbag.Compression.BZ2)
        for topic, msg, t in inbagfile.read_messages():
            if topic in to_keep:
                outbagfile.write(topic, msg, t)
        outbagfile.close()
        inbagfile.close()