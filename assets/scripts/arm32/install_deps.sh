if [ "$1" = "reset_image" ] ;
then
  ./reset_image.sh
else
  ./set_iot.sh
  ./reset_image.sh
fi