import cv2
import json
# "argparse" IS AN UNNEEDED IMPORT, JUST USED FOR CLI
import argparse

def parse(json_data, video_parser):
    # some metadata the MP4 stores that you can grab
    print(f"video FPS is {video_parser.get(cv2.CAP_PROP_FPS)}, {video_parser.get(cv2.CAP_PROP_FRAME_WIDTH)}x{video_parser.get(cv2.CAP_PROP_FRAME_HEIGHT)} (width x height)")

    for entry in json_data:
        # simple method for grabbing the data within a JSON entry
        relative_time = entry["relative_time"]
        angular_z = entry["angular_z"]
        linear_x = entry["linear_x"]
        frame_index = entry["frame_index"]

        # moving the current frame to be "frame_index"
        video_parser.set(cv2.CAP_PROP_POS_FRAMES, frame_index)

        ret, frame = video_parser.read()
        if not ret:
            print(f"Failed to read frame {frame_index}")
            continue

        # display the current frame in a window
        cv2.imshow("current_frame", frame)
        print(entry)
        cv2.waitKey(1)

    video_parser.release()
    cv2.destroyAllWindows()


def main():
    # just some argument stuff.
    parser = argparse.ArgumentParser()
    # python parser_example.py json_file_name.json mp4_file_name.mp4
    parser.add_argument("json", type=str)
    parser.add_argument("mp4", type=str)
    args = parser.parse_args()




    # loading the files
    with open(args.json) as f:
        json_data = json.load(f)
    video_parser = cv2.VideoCapture(args.mp4)

    # parsing the JSON and the video
    parse(json_data, video_parser)

if __name__ == "__main__":
    main()