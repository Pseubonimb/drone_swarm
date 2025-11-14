import os
import subprocess
import sys

# Имя виртуального окружения
venv_name = "drone_env"

# Создание виртуального окружения
print(f" Создаём виртуальное окружение '{venv_name}'...")
subprocess.check_call([sys.executable, "-m", "venv", venv_name])

# Активация виртуального окружения
if os.name == 'posix':
    activate_script = os.path.join(venv_name, 'bin', 'activate')
    pip_executable = os.path.join(venv_name, 'bin', 'pip3')
    python_executable = os.path.join(venv_name, 'bin', 'python3')
    path_export = f"export PATH=\"$PATH:$HOME/.local/bin\""
else:
    activate_script = os.path.join(venv_name, 'Scripts', 'activate')
    pip_executable = os.path.join(venv_name, 'Scripts', 'pip')
    python_executable = os.path.join(venv_name, 'Scripts', 'python')
    path_export = f"set PATH=\"%PATH%;%USERPROFILE%\\.local\\bin\""
    print("[!] Этот скрипт частично адаптирован для Windows, но рекомендуется использовать Linux.")

# Функция для вызова команд с активированным окружением
def run_in_venv(command):
    if os.name == 'posix':
        full_command = f"source {activate_script} && {command}"
        subprocess.check_call(full_command, shell=True, executable="/bin/bash")
    else:
        full_command = f"call {activate_script} && {command}"
        subprocess.check_call(full_command, shell=True)

print("Устанавливаем основные библиотеки через pip...")
packages = [
    "pymavlink",
    "pexpect",
    "empy==3.3.4",
    "dronecan",
    "setuptools",
    "PyYAML",
    "numpy",
    "matplotlib",
    "pyserial",
    "future",
    "lxml",
    "wxPython"
]
run_in_venv(f"{pip_executable} install {' '.join(packages)}")

print("Клонируем MAVProxy из GitHub...")
if not os.path.exists("MAVProxy"):
    subprocess.check_call(["git", "clone", "https://github.com/ArduPilot/MAVProxy.git"])
else:
    print("Папка MAVProxy уже существует, пропускаем клонирование.")

print("Устанавливаем MAVProxy...")
#run_in_venv(f"cd MAVProxy && {pip_executable} install -e .")
subprocess.check_call([python_executable, "-m", "pip", "install", "-e", "./MAVProxy"])

# Добавляем путь в .bashrc (только для Unix-систем)
bashrc_path = os.path.expanduser("~/.bashrc")
if os.name == 'posix' and os.path.exists(bashrc_path):
    with open(bashrc_path, "r") as f:
        if path_export not in f.read():
            print("Добавляем путь ~/.local/bin в PATH...")
            with open(bashrc_path, "a") as f_append:
                f_append.write(f"\n{path_export}\n")
else:
    print("[ERROR] не удалось обновить bashrc (Windows или файл отсутствует).")

print("\n Настройка завершена!")
print(f"Чтобы начать работать, активируйте окружение:\nsource {activate_script}")