# standard workflow
The procedures of manipulation can be devided into four parts in order:
 - **segmentation**: predicts the positions of objects in images.
 - **recognition**: predicts the objects' clasfication.
 - **picking**: predicts the pose of objects for picking.
 - **planning**: planning a collision-free trajetory from picking pose to target pose.

As some algorithms involve more than one part, a **end2end** folder is built for those algorithms. With a combination of the four functions, different tasks can be implemented. And with the standard workflow, we can compare the performance of different hardware and algorithms with designed metrics.

If you want to add a new module, you can fine a tutorial [here]([https://github.com/ancorasir/DeepClaw/blob/master/docs/Add%20New%20Module%20in%20Server.md]).