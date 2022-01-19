### Docker instructions

```
1) Clone the repo 
$ git clone https://github.com/alex-petrenko/megaverse.git

2) Build the image
cd megaverse
docker build -t megaverse -f docker/Dockerfile.base .

3) Start megaverse
docker run --shm-size 8G --runtime=nvidia megaverse ./docker/run.sh

(Optional) 4) Launch bash in the container and enjoy
docker run -it --shm-size 8G --runtime=nvidia --entrypoint /bin/bash megaverse
```