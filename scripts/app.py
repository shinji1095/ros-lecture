#!/usr/bin/env python3

import os
import rospy
import datetime
from linebot import LineBotApi, WebhookHandler
from flask import request, abort, Blueprint,jsonify, Flask
from linebot.exceptions import InvalidSignatureError,LineBotApiError
from linebot.models import MessageEvent, TextMessage, TextSendMessage

app = Flask(__name__)

# -------------------------- LINE Bot Init --------------------------
line_bot_api = LineBotApi(os.environ.get('CHANNEL_ACCESS_TOKEN'))
handler = WebhookHandler(os.environ.get('CHANNEL_SECRET'))


# -------------------------- Test Route --------------------------
@app.route('/')
def test():
    return jsonify({'message':'statusOK'}), 200


# -------------------------- LINE Bot Callback --------------------------
@app.route("/linebot/callback", methods=['POST'])
def callback():
    # get X-Line-Signature header value
    signature = request.headers['X-Line-Signature']

    # get request body as text
    body = request.get_data(as_text=True)
    app.logger.info("Request body: " + body)

    # handle webhook body
    try:
        handler.handle(body, signature)
    except InvalidSignatureError:
        print("Invalid signature. Please check your channel access token/channel secret.")
        abort(400)

    return jsonify({'message':'statusOK'}), 200


@handler.add(MessageEvent, message=TextMessage)
def handle_message(event):
    # -------------------------- ユーザ情報の取得 --------------------------
    profile = line_bot_api.get_profile(event.source.user_id)

    # -------------------------- メッセージによって処理を分岐 --------------------------
    color = event.message.text
    print(f'Tracking color: {color}')
    path = './tracking_color.txt'
    f = open(path, 'w')
    f.write(color)
    f.close()
    

    # -------------------------- 返信 --------------------------
    line_bot_api.reply_message(
        event.reply_token,
        TextSendMessage(text=f'Tracking color: {color}'))
    
    return jsonify({'message':'statusOK'}), 200

if __name__ == '__main__':
    rospy.init_node('flask_server', anonymous=False)
    app.run()
