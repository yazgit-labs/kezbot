#!/bin/bash 


SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

sudo cp $(rospack find mirBeta_udev)/udev/02-mirBeta.rules /etc/udev/rules.d/
ROBOT_NAME=""
read -e -p "Enter robot name (no '-' is allowed): " -i "mirBeta" ROBOT_NAME
sudo echo "#!/bin/bash
export ROBOT_NAME=$ROBOT_NAME
" > /etc/profile.d/mirBeta.sh

if [ "$?" -ne 0 ] ; then
  printf "%s\n" "${red}Env variables has not set." >&2
else
  printf "%s\n" "${green}Success.."
fi
