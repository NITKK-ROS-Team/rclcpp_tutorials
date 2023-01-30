# インストール（Docker・Ubuntu）

Ubuntu上のDockerへのインストールを行います。

## Dockerとは

Dockerとは、コンテナ型の仮想環境を提供するソフトウェアです。

コンテナ型の仮想環境とは、仮想マシンとは異なり、ホストOSのカーネルを共有することで、軽量な仮想環境を実現する技術です。Dockerを利用することで、ROS2の開発環境を簡単に構築することができます。

<br>

Dockerは元々IT分野におけるDevOps（継続的に顧客に価値を届けるために人、プロセス、テクノロジーを一つにまとめて開発と運用を行うこと）のために開発された概念で、主にアプリケーションのデプロイを容易にするために使われています。

コンピュータの環境に依存しにくい点は他のアプリケーションでも有用であるため、ROS2の開発環境を構築するためにも利用されています。

<!-- TODO: Dockerのわかりやすいイラストなど -->

<br>

## 要件

- Intel/AMD製CPUが搭載されたPC
- Jetson（NVIDIA L4T Baseに対応しているもの）

<br>

## Dockerのインストール

- [Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
- [Debian](https://docs.docker.com/engine/install/debian/)

新しいターミナル上で次のコマンドを実行して、Dockerをインストールします。

```bash
# Dockerのインストール
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release

# 既にインストールされている場合、
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor --yes -o /usr/share/keyrings/docker-archive-keyring.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io
```

<br>

インストールを行った直後のDockerは、管理者権限を持たないユーザーからは実行できません。そのため、管理者権限を持つユーザーにDockerを実行できるようにします。

```bash
# 管理者権限を付与
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

管理者権限の適用はログアウトすることで完全に反映されます。

<br>

## ROS2のインストール

<!-- CLIベース -->

<!-- GUIベース -->

<br>