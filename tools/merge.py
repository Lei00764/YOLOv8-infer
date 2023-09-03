import cv2
import numpy as np
import random
import math

def rotate_image(image, angle):
    # 获取旋转的中心点
    height, width = image.shape[:2]
    image_center = (width/2, height/2)
    
    # 计算旋转角度的弧度
    angle_rad = math.radians(angle)
    
    # 计算新图像的尺寸
    new_width = abs(np.sin(angle_rad)*height) + abs(np.cos(angle_rad)*width)
    new_height = abs(np.sin(angle_rad)*width) + abs(np.cos(angle_rad)*height)

    # 获取旋转矩阵
    rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1)
    
    # 更新旋转矩阵的偏移量
    rotation_mat[0, 2] += (new_width - width) / 2
    rotation_mat[1, 2] += (new_height - height) / 2

    # 应用旋转
    rotated_img = cv2.warpAffine(image, rotation_mat, (int(new_width), int(new_height)))
    return rotated_img

# 加载模板图片
template_img = cv2.imread('mask1.png', cv2.IMREAD_UNCHANGED)

# 随机生成旋转角度
angle = random.uniform(-30, 30)

# 对图片进行旋转
rotated_img = rotate_image(template_img, angle)

# 获取旋转后的图片尺寸
rotated_height, rotated_width = rotated_img.shape[:2]

# 读取荔枝纹浅灰色PVC塑胶地的背景图片
background_img = cv2.imread('background.png')

# 调整背景图片的尺寸以匹配旋转后的图片
background_img = cv2.resize(background_img, (rotated_width, rotated_height))

# 提取遮罩图像的Alpha通道
mask_alpha = rotated_img[:, :, 3]

# 将Alpha通道转换为3通道
mask_alpha = cv2.cvtColor(mask_alpha, cv2.COLOR_GRAY2BGR)

# 将Alpha通道进行归一化
mask_alpha = mask_alpha.astype(float) / 255

# 将模板图像的RGB通道与Alpha通道进行分离
rotated_rgb = rotated_img[:, :, :3]

# 对背景图像和模板图像进行透明融合
result_img = (1 - mask_alpha) * background_img + rotated_rgb * mask_alpha
result_img = result_img.astype(np.uint8)

# 计算新的尺寸
new_height = int(rotated_height * 1.2)
new_width = int(rotated_width * 1.2)

# 读取荔枝纹浅灰色PVC塑胶地的背景图片
extended_img = cv2.imread('background.png')

# 调整背景图片的尺寸以匹配新的尺寸
extended_img = cv2.resize(extended_img, (new_width, new_height))

# 将融合后的图片放置到新的背景图片中心
start_y = (new_height - rotated_height) // 2
start_x = (new_width - rotated_width) // 2
extended_img[start_y:start_y+rotated_height, start_x:start_x+rotated_width] = result_img

# 显示结果
cv2.imshow('Result', extended_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
