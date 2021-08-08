function getAWSCredentials() {
    var awsConfiguration = {
        host: "a3bxp6hd5fzqdr-ats.iot.us-east-1.amazonaws.com", // AWS MQTT Endpoint  <make sure add -ats in the prefix of the end point (or before .iot)
        region: "us-east-1", // AWS Region, ie. us-east-1
        accessKeyId: "ASIA5XIA6X4V7FWJWAHY", // AWS IAM User's Access Key
        secretAccessKey: "gdoxWNZ9EBs73qIobeksyDL2Iz4JOmwLf7TdftDV", // AWS IAM User's Secret Key
        sessionToken: "FwoGZXIvYXdzEIj//////////wEaDFn1lgslcHJdy6MAgiLFATD0806aX26j2uhd6MDzAwr7gmhCe1oY5sdkt9sUCR6OHF+djpU9Ez8evqI9QMuYT09kaGTu/EXwFBNmmUwpu9fEZ5zotSuB3Y0lA4f6dp0I4qIytomksRZla+oXPcZWkHacVrQbXp5eR2vaVoBu04CdGh/7/OMWiCAPSd5aR91Iyc0/AnvkIEzMlvq3y3r9DhEw1nSlXTcCdh9N1Wy2NQ6rpBykoJ+UWLgaJ2itEn0slOT3u97dxbYFrRf1kjMDxP7ChCKAKOjdv4gGMi0YtyBqbcrFJW+1R7xk7wv3AtmrIczlA1eTe9vQ09f5850xxuKb9aC1KvpGOGU"
    };

     return awsConfiguration;
}
