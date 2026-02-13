Для установки среды имитационного моделирования предварительно необходимо:
1. Установить ОБЯЗАТЕЛЬНО Webots с официального репозитория https://github.com/cyberbotics/webots/releases?spm=a2ty_o01.29997173.0.0.54b151715qf3yJ <br>
2. глобально установить: <br>
`sudo apt update`
`sudo apt install -y \`
`    python3-dev \`
`    libgtk-3-dev \`
`    libwebkit2gtk-4.1-dev \`
`    libgstreamer-plugins-base1.0-dev \`
`    gstreamer1.0-plugins-base \`
`    libgstreamer1.0-dev \`
`    libgstreamer-plugins-good1.0-dev \`
`    libjpeg-dev \`
`    libpng-dev \`
`    libtiff-dev \`
`    libsdl2-dev`
<br>
3. В папке, где мы хотим создать проект, открываем консоль и запускаем развёртку виртуального окружения командой: <br>
`python3 setup_env.py` <br>
4. Далее активируем виртуальное окружение командой: <br>
`source drone_env/bin/activate` <br>
5. После чего можем запускать мир с нужным количеством дронов (данная строка запустит мир с тремя дронами): <br>
`python3 run_sim.py --drones 3` <br>
