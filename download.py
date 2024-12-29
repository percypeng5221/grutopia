import openxlab
openxlab.login(ak="xvw9klyr0dq53n8p2y7a", sk="wpw7kbxkeqbjlzdneq7xwbdgpndq4g2lpgv90zer") # 进行登录，输入对应的AK/SK，可在个人中心添加AK/SK


from openxlab.dataset import get
get(dataset_repo='OpenRobotLab/GRScenes', target_path='.') # 数据集下载
