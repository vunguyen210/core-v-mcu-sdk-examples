import json
import boto3
import requests
from enum import Enum
from aws_requests_auth.aws_auth import AWSRequestsAuth
from urllib.parse import quote
import os
import sys

STATUS = Enum("STATUS", "PASS, FAIL, ERROR, TIMEOUT")

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)

class AWSOperations:
    """
    Creates a session using AWS SDK to call the API gateway to register the device.
    This class reads the aws_configs.json to get the necessary credentials to 
    connect to AWS.
    """

    IDENTITY_POOL_ID:str = None
    IDENTITY_ID:str = None 
    ACCOUNT_ID:str = None
    API_HOST:str = None
    IOT_ENDPOINT:str = None
    API_METHOD:str = 'getToken'

    def __init__(self, vendor_name, stage) -> None:
        self.vendor_name = vendor_name
        self.stage = stage
        with open(resource_path("aws_config.json"), "r") as conf:
            data = json.loads(conf.read())
            self.IDENTITY_POOL_ID = data[vendor_name][stage]["IDENTITY_POOL_ID"]
            self.ACCOUNT_ID = data[vendor_name][stage]["ACCOUNT_ID"]
            self.API_HOST = data[vendor_name][stage]["API_HOST"]
            self.IOT_ENDPOINT = data[vendor_name][stage]["IOT_ENDPOINT"]    
    
    def start_aws_session(self):   
        """
        Create a Federated Identity using the unauthenticated cognito pool and start
        a Session with AWS.
        """
        print('Starting AWS Session')

        try:    
            self.cognito_client = boto3.client('cognito-identity', region_name='us-west-2')

            # Generate a new Federated Identity.
            response = self.cognito_client.get_id(AccountId=self.ACCOUNT_ID, IdentityPoolId=self.IDENTITY_POOL_ID)
            self.IDENTITY_ID = response['IdentityId']
            # Start a Session with AWS using Identity credentials.
            response = self.cognito_client.get_credentials_for_identity(IdentityId=self.IDENTITY_ID)
            access_key = response['Credentials']['AccessKeyId']
            secret_key = response['Credentials']['SecretKey']
            session_token = response['Credentials']['SessionToken']
            self.active_session = boto3.session.Session(
                aws_access_key_id = access_key,
                aws_secret_access_key = secret_key,
                aws_session_token = session_token,
                region_name = 'us-west-2'
            )
        except Exception as e:
            return STATUS.ERROR

        print('AWS Session Initialized\n')
        return STATUS.PASS

    def call_api_gateway(self, thing_name, certificate):
        """
        Call the API-GW with the thing name and device certificate extracted from
        the platform.
        """
        demo_status = self.start_aws_session()
        name = thing_name
        hex_flag = True
        if demo_status != STATUS.PASS:
            return demo_status, ''

        credentials = self.active_session.get_credentials()
        auth = AWSRequestsAuth(aws_access_key=credentials.access_key,
                            aws_secret_access_key=credentials.secret_key,
                            aws_token=credentials.token,
                            aws_host=self.API_HOST,
                            aws_region='us-west-2',
                            aws_service='execute-api')

        body = json.dumps({'thingName':name}) 
        headers = {"Content-type": "application/json"}
        response = requests.post(f'https://{self.API_HOST}/{self.API_METHOD}', auth=auth, data=body, headers=headers)        
        if response.status_code != 200:
            try: 
                name = str(int(name, base=16))
            except Exception as err:
                hex_flag = False
            
            if hex_flag :
                credentials = self.active_session.get_credentials()
                auth = AWSRequestsAuth(aws_access_key=credentials.access_key,
                                    aws_secret_access_key=credentials.secret_key,
                                    aws_token=credentials.token,
                                    aws_host=self.API_HOST,
                                    aws_region='us-west-2',
                                    aws_service='execute-api')
                body = json.dumps({'thingName':name}) 
                headers = {"Content-type": "application/json"}
                response = requests.post(f'https://{self.API_HOST}/{self.API_METHOD}', auth=auth, data=body, headers=headers)        
            
            if response.status_code != 200:
                print(f'Something went wrong while processing the request. Response: {response.text}, Status code: {response.status_code}')
                return STATUS.ERROR, ''
        print()
        
        return STATUS.PASS, json.loads(response.text)["token"], name

    def get_endpoint(self) -> str:
        return self.IOT_ENDPOINT

def main():
    aws_ops = AWSOperations("EL", "prod")
    cert = ""
    name = input('Please enter your device thing Name from printed at the UART: ')

    demo_status, user_token, name = aws_ops.call_api_gateway(name, cert) 
    if demo_status is STATUS.PASS:
        user_token = quote(user_token,safe='')
        print(user_token)
        base_url = f'https://iot-expresslink.us-west-2.amazonaws.com/?thingName=EL?boardName={name}?token={user_token}'
        print(base_url)
        print("Please use the above link to access the vizualizer while making sure the demo code is running.")
        
main()
