          

# ubuntu安装NVIDIA显卡驱动

1 更新软件源

2查看显卡信息

系统信息：
```
lsb_release -a

lspci | grep -i nvidia

```
显卡是否生效

`lshw -c video`

查看显存：

`lspci -v -s 01:00.0`

3 下载显卡驱动

4 禁用nouveau

1
`vim /etc/modprobe.d/blacklist.conf`

2 末尾添加

```
blacklist nouveau

options nouveau modeset=0

lsmod | grep nouveau

sudo update-initramfs -u

reboot
```
5 安装显卡驱动  
 
 一直默认就行