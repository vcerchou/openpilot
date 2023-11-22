简化安装(体积最小)
---
```bash
cd /data/ && \
mv openpilot op_backup_$(date +%Y%m%d%H%M%S) ; \
git clone https://github.com/vcerchou/openpilot.git openpilot -b M3-UI --single-branch --depth=1 && \
cd openpilot/ && \
git pull && \
reboot
```

完整安装(所有分支)
---
```bash
cd /data/ && \
mv openpilot op_backup_$(date +%Y%m%d%H%M%S) ; \
git clone https://github.com/vcerchou/openpilot.git openpilot -b M3-UI && \
cd openpilot/ && \
git pull && \
reboot
```

安装
---

1. SSH 到你的设备

2. 切换到 /data/ directory 
    ```bash
    cd /data
    ```
3. 备份openpilot (可选的)
    ```bash
    mv openpilot op_backup_$(date +%Y%m%d%H%M%S)
    ```
4. 从github仓库下载分支:
    * github 
    ```bash
    git clone https://github.com/vcerchou/openpilot.git openpilot -b M3-UI --single-branch --depth=1
    ```
5. 切换到openpilot目录：
    ```bash
    cd openpilot
    ```
6. 再拉取一次：
    ```bash
    git pull
    ```
7.  重启
    ```bash
    reboot
    ```
8.  如果拉取失败则执行：
    ```bash
    git fetch --all
    git reset --hard origin/分支名称
    git pull
    ```