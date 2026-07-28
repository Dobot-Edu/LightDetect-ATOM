import requests
import json
import os
from voice_sdk.realsense_camera import RealSenseCamera


VLM_USER_ID = "user123"
VLM_API_KEY = "app-VD5kXGNd2FLDASe4cCVEjCva"
VLM_BASE_URL = "http://192.168.8.13/v1"

class VLM:
    def __init__(self, api_url, api_key, cam):
        self.camera = cam
        self.api_url = api_url
        self.api_key = api_key
        self.user_id = VLM_USER_ID
        self.headers = {'Authorization': f'Bearer {self.api_key}',
                        'Content-Type': 'application/json'}
        self.file_path = os.path.dirname(__file__)
        self.img_abs_path = self.file_path + "/data/shot.jpg"
        self.conversations_id = None

    def delete(self, conver_id):
        url = f'{self.api_url}/conversations/{conver_id}'
        data = {
            "user": self.user_id
        }
        response = requests.delete(url, json=json.dumps(data), headers=self.headers)
        print("response: ", response.json(), response)
        return response.json()

    def upload_file(self, file_path):
        url = f'{self.api_url}/files/upload'
        files = {'file': ("a.jpg", open(file_path, 'rb'), 'image/jpg')}
        headers = {
            'Authorization': f'Bearer {self.api_key}',
            "user": self.user_id
        }
        response = requests.post(url, files=files, headers=headers)
        self.conversations_id = response.json()["id"]
        print("upload_file: ", self.conversations_id)
        return response.json()

    def forward(self, inp_txt):
        print("self.img_abs_path: ", self.img_abs_path)
        self.camera.shot_img(self.img_abs_path)
        ret = self.upload_file(self.img_abs_path)
        file_id = ret["id"]
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        payload = {
            "inputs": {},
            "query": str(inp_txt),
            "files": [{
                "type": "image",
                "transfer_method": "local_file",
                "upload_file_id": file_id
            }],
            "response_mode": "blocking",
            "user": "usr123"
        }
        response = requests.post(
            f"{self.api_url}/chat-messages",
            headers=headers,
            data=json.dumps(payload),
            stream=False
        )
        # print(response.content)
        rt = json.loads(response.content.decode("utf-8"))["answer"].strip()
        self.delete(self.conversations_id)
        return rt


class LLM:
    def __init__(self, api_url, api_key):
        self.api_url = api_url
        self.api_key = api_key
        self.user_id = VLM_USER_ID
        self.headers = {'Authorization': f'Bearer {self.api_key}',
                        'Content-Type': 'application/json'}


    def forward(self, inp_txt):
        SUPPORT_LANGUAGE = {"en": "英文", "ch": "中文", "ja": "日语", "ko": "韩语"}
        lang = "ch"
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        payload = {
            "inputs": {},
            "query": str(inp_txt) + f"(回答请用{SUPPORT_LANGUAGE[lang]})",
            "response_mode": "blocking",
            "user": "user123"
        }
        response = requests.post(
            self.api_url,
            headers=headers,
            data=json.dumps(payload),
            stream=False
        )
        rt = json.loads(response.content.decode("utf-8"))["answer"].strip()
        print(rt)
        return rt


if __name__ == "__main__":
    # aaa = VLM(api_url="http://192.168.8.13/v1",
    #                       api_key="app-VD5kXGNd2FLDASe4cCVEjCva",
    #                       cam=RealSenseCamera(flip=True, device_id="130322273839"))
    # aaa.delete("57afd0f6-444e-476b-a754-4522bebd58a6")
    aaa = LLM(api_url="http://192.168.8.13/v1/chat-messages",
              api_key="app-iDcz4WaDSzTXjIowQt7dvbYl")
    aaa.forward("who are you?")