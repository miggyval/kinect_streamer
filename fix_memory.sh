sudo sh -c 'echo 128 > /sys/module/usbcore/parameters/usbfs_memory_mb'
echo usbfs memory set to: 
cat /sys/module/usbcore/parameters/usbfs_memory_mb
