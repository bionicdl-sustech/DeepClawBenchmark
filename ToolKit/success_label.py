import DetectForeground as df
def success_label(imgray,color_image):
    imgray = imgray[500:700,120:300]

    imagegray = color_image[500:700,120:300]

    compare = df.Segment()
    rect = compare.DiffGround(imagegray,imgray)
    if len(rect) > 0 :
        success_label = 1
        print('success_label:'+str(success_label))
        return success_label, imagegray
    else :
        success_label = 0
        print('success_label:'+str(success_label))

        return success_label, imagegray

if __name__=="__main__":
    a = success_label()
    print(a)