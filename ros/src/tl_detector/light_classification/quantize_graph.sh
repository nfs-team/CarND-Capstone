# Quantize weights to reduce file size
python -m tensorflow.tools.graph_transforms.transform_graph \
--in_graph=frozen_models/frozen_sim_inception/optimized_ssd_graph.pb \
--out_graph=frozen_models/frozen_sim_inception/eightbit_graph.pb \
--input_names=image_tensor \
--output_names=detection_boxes,detection_scores,detection_classes,num_detections \
--transforms='
add_default_attributes
remove_nodes(op=Identity, op=CheckNumerics)
fold_constants(ignore_errors=true)
fold_batch_norms
fold_old_batch_norms
fuse_resize_and_conv
quantize_weights
quantize_nodes
strip_unused_nodes
sort_by_execution_order'