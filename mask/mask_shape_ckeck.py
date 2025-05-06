#检测mask图片的shape
# 读取mask图片
import cv2

mask= cv2.imread("mask/mask_0.png", cv2.IMREAD_UNCHANGED)
# 获取mask图片的shape
shape = mask.shape
# 输出mask图片的shape
print("mask shape:", shape)

# 显示mask图片
cv2.imshow("mask", mask)
# 等待按键事件
cv2.waitKey(0)
