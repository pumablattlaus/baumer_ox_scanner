# Project Title

## Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Usage](#usage)
- [Contributing](../CONTRIBUTING.md)

## About <a name = "about"></a>

Write about 1-2 paragraphs describing the purpose of your project.

## Getting Started <a name = "getting_started"></a>

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See [deployment](#deployment) for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them.

```
Give examples
```

### Installing
#### C++
Installing from LibOxApi_02-00-00.tar.gz:

```bash
    cd ~/Downloads
    tar -xvzf LibOxApi_02-00-00.tar.gz
    cd LibOxApi_02-00-00
    sudo cp -ri include/* /usr/local/include/
    sudo cp -ri lib/* /usr/local/lib/
```

Add lib to path (or in bashrc):

```bash
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

#### API via python for ubuntu
Not working!

Install .NET for Linux:

```bash
    wget https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
```

```bash
    sudo dpkg -i packages-microsoft-prod.deb
    rm packages-microsoft-prod.deb
```

```bash
    sudo apt update
    # sudo apt-get install -y apt-transport-https && \
    # sudo apt-get update && \
    sudo apt install -y dotnet-sdk-8.0
```


Install Mono for Linux:

```bash
sudo apt install ca-certificates gnupg
sudo gpg --homedir /tmp --no-default-keyring --keyring /usr/share/keyrings/mono-official-archive-keyring.gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
echo "deb [signed-by=/usr/share/keyrings/mono-official-archive-keyring.gpg] https://download.mono-project.com/repo/ubuntu stable-focal main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
sudo apt update
sudo apt install mono-devel
```


End with an example of getting some data out of the system or using it for a little demo.

## Usage <a name = "usage"></a>

Add notes about how to use the system.
