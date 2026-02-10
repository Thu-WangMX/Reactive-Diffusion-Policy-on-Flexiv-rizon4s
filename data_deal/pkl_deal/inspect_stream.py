import pickle
import os
import sys

# 替换为那个“空”文件的路径
BAD_FILE_PATH = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/plug_in_usb/seq_00021.pkl'

def inspect_stream():
    if not os.path.exists(BAD_FILE_PATH):
        print("File not found.")
        return

    file_size = os.path.getsize(BAD_FILE_PATH)
    print(f"File Size on Disk: {file_size / 1024 / 1024:.2f} MB")

    with open(BAD_FILE_PATH, 'rb') as f:
        # 1. 读取第一个对象（目前已知是空的）
        print("\n--- Attempting Load 1 ---")
        try:
            obj1 = pickle.load(f)
            pos1 = f.tell()
            print(f"Load 1 Success. Object Type: {type(obj1)}")
            
            # 检查里面有多少数据
            length = 0
            if hasattr(obj1, 'sensorMessages'):
                length = len(obj1.sensorMessages)
            print(f"Data length in Obj1: {length}")
            
            print(f"File pointer position: {pos1} bytes")
            print(f"Percentage of file read: {pos1 / file_size * 100:.4f}%")
            
            if pos1 < file_size:
                print("\n[DETECTED] There is unread data remaining in the file!")
            else:
                print("\n[WEIRD] File pointer is at end, but data was empty? This implies the 1GB is junk/zeros.")
                return

        except Exception as e:
            print(f"Load 1 Failed: {e}")
            return

        # 2. 尝试读取第二个对象（看看是不是散落的帧）
        print("\n--- Attempting Load 2 (The Hidden Data) ---")
        try:
            obj2 = pickle.load(f)
            print(f"Load 2 Success! Object Type: {type(obj2)}")
            # 看看这是不是一帧数据
            print(f"Object details: {obj2}")
            
            # 如果成功，尝试循环读取以此统计总帧数
            print("\n--- Counting remaining frames... ---")
            count = 1 # 刚才读了一个
            while True:
                try:
                    pickle.load(f)
                    count += 1
                    if count % 100 == 0:
                        sys.stdout.write(f"\rFound {count} frames so far...")
                        sys.stdout.flush()
                except EOFError:
                    print(f"\nFinished. Total extra frames found: {count}")
                    break
                except Exception as e:
                    print(f"\nStopped due to error: {e}")
                    break
        except EOFError:
            print("End of file reached immediately.")
        except Exception as e:
            print(f"Load 2 Failed: {e}")

if __name__ == '__main__':
    inspect_stream()