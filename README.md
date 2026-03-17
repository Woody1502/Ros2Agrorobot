yay -S nvidia-container-toolkit
Затем настройте Docker runtime:


sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
Проверьте что работает:


docker run --rm --gpus all ubuntu nvidia-smi
После этого:

xhost +local:docker
docker compose up