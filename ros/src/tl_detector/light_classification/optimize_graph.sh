# Optimize tensorflow graph for inference

python -m tensorflow.python.tools.optimize_for_inference \
--input=frozen_models/frozen_sim_inception/frozen_inference_graph.pb \
--output=frozen_models/frozen_sim_inception/optimized_ssd_graph.pb \
--frozen_graph=True \
--input_names=image_tensor \
--output_names=detection_boxes,detection_scores,detection_classes,num_detections

python -m tensorflow.python.tools.optimize_for_inference \
--input=frozen_models/frozen_real_inception/frozen_inference_graph.pb \
--output=frozen_models/frozen_real_inception/optimized_ssd_graph.pb \
--frozen_graph=True \
--input_names=image_tensor \
--output_names=detection_boxes,detection_scores,detection_classes,num_detections