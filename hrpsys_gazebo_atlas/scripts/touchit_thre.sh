#!/bin/bash 

thre=20
echo $#
echo $1
if [ $# -eq 1 ]
then
thre=$1
fi

echo -e "\e[1;31mset touchit/thre $thre\e[m"
rostopic pub /touchit/thre std_msgs/Float64 $thre -1

