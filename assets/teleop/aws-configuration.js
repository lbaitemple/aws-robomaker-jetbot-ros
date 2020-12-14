function getAWSCredentials() {
    var awsConfiguration = {
        host: "a38zt1esnt42xy-ats.iot.us-east-1.amazonaws.com", // AWS MQTT Endpoint  <make sure add -ats in the prefix of the end point (or before .iot)
        region: "us-east-1", // AWS Region, ie. us-east-1
        accessKeyId: "ASIA54JKYWY26UXUXLCQ", // AWS IAM User's Access Key
        secretAccessKey: "7nyt6fPeaHw8grJYSu4x6qo8mQfdwN4i3InGr8df", // AWS IAM User's Secret Key
        sessionToken: "FwoGZXIvYXdzEE8aDHac1WGi7Y3CSXEdayK+AZDSZOqe1uRrFJR/rUmTYtSx3oorUxI5XkrhdomhcAq8arRVZGiQDcy9ex9xw2AyJxLxfi/3gYuuuq+garvYR9o1q83tVdDhaKtzmQxgwLr4ZgwP7t4F+eCSWogPeFKtQ9EnujcaDQJuUjZs9xVL3Yj6+PfIiiHv2CzsX7pmbDtG5+CeuCreEQF8rnuAmrQvehbpEk+fWfIS4V91OtQl+HZAKKd7DNkRGSNw5y71RMp0Q8BzzKJK/QUy+r94HoMoqNzd/gUyLTluJxOZpt+2EuUHLH10zeHcTsqVtIv1BtDAg3yfuranTMGpH/h5DoqC+8ecwQ=="
    };

     return awsConfiguration;
}
