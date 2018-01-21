from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2
import numpy as np


class TLClassifier(object):



    def __init__(self,fx,fy,image_width,image_height,scenes):
        self.fx = fx
        self.fy = fy
        self.image_width = image_width
        self.image_height = image_height

        model_data = ['sim.pb','site.pb']
        self.min_score_thresh = 0.7

        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(model_data[scenes], 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        with detection_graph.as_default():
            self.sess = tf.Session(graph=detection_graph)
            # Definite input and output Tensors for detection_graph
            self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_np_expanded = np.expand_dims(image, axis=0)


        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        red = [1,4,5,8,12,13]
        yellow = [6]
        green = [0,2,3,9,10,11]
        no_light = [7]

	light_state = TrafficLight.UNKNOWN

        for i in range(boxes.shape[0]):
            if scores[i] > self.min_score_thresh:
                #class_name = self.category_index[classes[i]]['name']
                #print('{}'.format(class_name), scores[i])
                #print(scores,i)
                print (i)
                #print(boxes[i][1],boxes[i][3])
                y1 = int(boxes[i][1]*self.image_width)
                y2 = int(boxes[i][3]*self.image_width)
                x1 = int(boxes[i][0]*self.image_height)
                x2 = int(boxes[i][2]*self.image_height)

                light = image[x1:x2,y1:y2,:]
                ex_cal_red = np.array(light[:,:,2])
                ex_cal_green = np.array(light[:,:,1])
                ex_cal_blue = np.array(light[:,:,0])

                ex_cal_red[ex_cal_red<100]=0
                ex_cal_red[ex_cal_red>200]=255
                ex_cal_green[ex_cal_green<100]=0
                ex_cal_green[ex_cal_green>200]=255
                ex_cal_blue[ex_cal_blue<100]=0
                ex_cal_blue[ex_cal_blue>200]=255

                ex_cal_red = ex_cal_red.mean()
                ex_cal_green = ex_cal_green.mean()
                ex_cal_blue = ex_cal_blue.mean()


                cv2.imshow("light"+str(i),light)
                cv2.waitKey(1)
                if (ex_cal_red-ex_cal_green)>5:
                    print("Red")
                    light_state = TrafficLight.RED
                elif (ex_cal_red-ex_cal_green)<-5:
                    print("Green")
                    light_state = TrafficLight.GREEN
                else:
                    print("Yellow")
                    light_state = TrafficLight.YELLOW
                print(ex_cal_red,ex_cal_green,ex_cal_blue)

        return light_state
