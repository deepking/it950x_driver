echo 'wmem_default ' `cat /proc/sys/net/core/wmem_default`
echo 'wmem_max ' `cat /proc/sys/net/core/wmem_max`
echo 'tcp_wmem ' `cat /proc/sys/net/ipv4/tcp_wmem`
echo ' '
echo 'rmem_default ' `cat /proc/sys/net/core/rmem_default`
echo 'rmem_max ' `cat /proc/sys/net/core/rmem_max`
echo 'tcp_rmem ' `cat /proc/sys/net/ipv4/tcp_rmem`
