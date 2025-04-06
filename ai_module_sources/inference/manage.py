import paramiko
from scp import SCPClient


def extract_pid(result: str):
    lines = result.strip().split('\n')

    for line in lines:
        if line.split()[2] == "/root/fruit_det":
            return int(line.split()[0])
        
    return None

def ssh_execute_command(hostname, username, password, commands: list, port: int = 22):
    try:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(hostname, port=port, username=username, password=password)

        for command in commands:
            print(f"Ready to exec: {command}")
            stdin, stdout, stderr = client.exec_command(command)
            output = stdout.read().decode()
            error = stderr.read().decode()
            return output
            

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        client.close()


def scp_send_file(hostname, username, password, local_path, remote_path, port: int = 22):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        ssh.connect(hostname, port=port, username=username, password=password)

        with SCPClient(ssh.get_transport()) as scp:
            print("TX'ing...")
            scp.put(local_path, remote_path)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ssh.close()
        print("Connection closed.")

class API:
    def __init__(self, host: str = "10.160.209.1", username: str = "root", password: str = "root"):
        self.host = host
        self.username = username
        self.password = password

    def update_weights(in_path: str, out_path: str = "/root/"): pass
    def board_camera_prepare(self):
        ssh_execute_command(
            hostname=self.host, 
            username=self.username,
            password=self.password,
            commands=["echo 255 | /mnt/system/usr/bin/sensor_test"]
        )

    def start_yolo(self):
        ssh_execute_command(
            hostname=self.host, 
            username=self.username,
            password=self.password,
            commands=["export LD_LIBRARY_PATH=/root/libs_patch/lib:/root/libs_patch/middleware_v2:/root/libs_patch/middleware_v2_3rd:/root/libs_patch/tpu_sdk_libs:/root/libs_patch:/root/libs_patch/opencv && nohup /root/fruit_det /root/fruit_2_int8_2_class.cvimodel 3 > /dev/null 2>&1 &"]
        )

    def stop_yolo(self, pid: int): pass

if __name__ == "__main__":
    # Example pipeline to start YOLO camera detection
    api = API()
    # api.board_camera_prepare()
    # api.start_yolo()
    pid = ssh_execute_command("10.160.209.1", "root", "root", ["ps aux | grep /root/fruit_det"])
    pid = extract_pid(pid)
    print(f"Pid: {pid}")

    # ssh_execute_command(
    #     hostname=host, # 
    #     username=username,
    #     password=password,
    #     commands=["touch /home/pi/connected_an_create"]
    # )

    # scp_send_file(
    #     hostname=host,
    #     username=username,
    #     password=password,
    #     local_path="/home/stephan/Downloads/bad_calib_last_dataset_fruits_model.cvimodel",
    #     remote_path="/home/pi/"
    # )