# Prerequisites

Docker and Docker compose installed
```
sudo apt install docker.io docker-compose
```

Pi user added to group docker
```
sudo usermod -a -G docker pi
```

Login again to let the group change take effect.

# Run
Change directory to the directory where this README.md is located.

```
docker-compose up
```
