from flask import  (
    request, 
    abort, 
    Blueprint,
    jsonify
    )


from linebot.exceptions import (
    InvalidSignatureError,
    LineBotApiError
)
from linebot.models import (
    MessageEvent, TextMessage, TextSendMessage,
)

from app import (
    line_bot_api,
    handler,
    db
)

import datetime


linebot_url = Blueprint("linebot", __name__, '/linebot')


# -------------------------- LINE Bot Callback --------------------------
@linebot_url.route("/callback", methods=['POST'])
def callback():
    # get X-Line-Signature header value
    signature = request.headers['X-Line-Signature']

    # get request body as text
    body = request.get_data(as_text=True)
    linebot_url.logger.info("Request body: " + body)

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
    print('name:', profile.display_name,', id:', profile.user_id)

    # -------------------------- メッセージによって処理を分岐 --------------------------
    message = event.message.text
    print(f"[*] message : {message}")
    if message == "価格は[0-9]+円":
        value = message.split('は')[-1].split('円')[0]
        print(f'[*] value : {value}')
        db.collection('recipt').add({
        'name': profile.display_name,
        'value': int(value),
        'timestamp':datetime.now()
    })

    # -------------------------- 返信 --------------------------
    line_bot_api.reply_message(
        event.reply_token,
        TextSendMessage(text='登録完了しました'))
    
    return jsonify({'message':'statusOK'}), 200