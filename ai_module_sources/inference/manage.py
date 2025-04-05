import paramiko
from scp import SCPClient

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
    def update_weights(in_path: str, out_path: str = "/"): pass
    def board_camera_prepare(): pass
    def start_yolo(): pass
    def stop_yolo(): pass

if __name__ == "__main__":
    host = "192.168.1.10"
    username = "pi"
    password = "pi"
    if 0:
        ssh_execute_command(
            hostname=host, # 10.160.209.1
            username=username,
            password=password,
            commands=["touch /home/pi/connected_an_create"]
        )

    scp_send_file(
        hostname=host,
        username=username,
        password=password,
        local_path="/home/stephan/Downloads/bad_calib_last_dataset_fruits_model.cvimodel",
        remote_path="/home/pi/"
    )