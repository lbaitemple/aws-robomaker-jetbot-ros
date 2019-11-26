function getAWSCredentials() {
    var awsConfiguration = {
        host: "<AWS IoT Endpoint URL ie. xxxxxxxxx.iot.us-west-2.amazonaws.com>", // AWS MQTT Endpoint 
        region: "<AWS region ie. us-east-2>", // AWS Region, ie. us-east-1
        accessKeyId: "<AWS IAM User's Access Key ie. AKIA2GZQ4XXXXXXXXXX>", // AWS IAM User's Access Key
        secretAccessKey: "<AWS IAM User's Secret Key ie. XXXXXXbmi7BQGaImE3uwW6bB>", // AWS IAM User's Secret Key
        sessionToken: null
     };

     return awsConfiguration;
}
