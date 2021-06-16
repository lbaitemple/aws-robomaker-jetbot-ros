WORK_DIR=$(pwd)

# Set variables
TELEOP_CRED=$WORK_DIR/../teleop/aws-configuration.js
TELEOP_LEFTOVER=$WORK_DIR/../teleop/aws-configuration.js.bak

# reload file
rm $TELEOP_CRED
cp ./resources/aws-configuration.js $TELEOP_CRED

# Create credentials
export host_ani=`aws iot describe-endpoint --endpoint-type iot:Data-ATS | grep \" | cut -d \" -f4`
export region_ani=`cat ~/.aws/credentials | grep region | cut -d \= -f2`

export accessKeyId_ani=`cat resources/credentials | grep aws_access_key_id | cut -d \= -f2`
export secretAccessKey_ani=`cat resources/credentials | grep aws_secret_access_key | cut -d \= -f2`
export sessionToken_ani=`cat resources/credentials | grep aws_session_token | cut -d \= -f2`

# Add credentials to teleop
perl -pi.bak -e 's/host: \"\"/host: \"$ENV{host_ani}\"/' $TELEOP_CRED
perl -pi.bak -e 's/region: \"\"/region: \"$ENV{region_ani}\"/' $TELEOP_CRED
perl -pi.bak -e 's/accessKeyId: \"\"/accessKeyId: \"$ENV{accessKeyId_ani}\"/' $TELEOP_CRED
perl -pi.bak -e 's/secretAccessKey: \"\"/secretAccessKey: \"$ENV{secretAccessKey_ani}\"/' $TELEOP_CRED
perl -pi.bak -e 's/sessionToken: \"\"/sessionToken: \"$ENV{sessionToken_ani}\"/' $TELEOP_CRED

# remove leftovers
rm $TELEOP_LEFTOVER