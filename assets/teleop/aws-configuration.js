function getAWSCredentials() {
    var awsConfiguration = {
        host: "", // AWS MQTT Endpoint  <make sure add -ats in the prefix of the end point (or before .iot)
        region: "us-east-1", // AWS Region, ie. us-east-1
        accessKeyId: "", // AWS IAM User's Access Key
        secretAccessKey: "", // AWS IAM User's Secret Key
        sessionToken: ""
    };

     return awsConfiguration;
}
