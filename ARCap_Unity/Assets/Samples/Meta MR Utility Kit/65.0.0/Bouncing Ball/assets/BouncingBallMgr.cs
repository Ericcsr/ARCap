/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using UnityEngine;

public class BouncingBallMgr : MonoBehaviour
{
    [SerializeField] private Transform trackingSpace;
    [SerializeField] private Transform rightControllerPivot;
    [SerializeField] private GameObject ballPrefab;

    private BouncingBallLogic currentBall;
    private bool ballGrabbed;

    private void Update()
    {
        const OVRInput.RawButton grabButton = OVRInput.RawButton.RHandTrigger;
        if (!ballGrabbed && OVRInput.GetDown(grabButton))
        {
            currentBall = Instantiate(ballPrefab).GetComponent<BouncingBallLogic>();
            ballGrabbed = true;
        }

        if (ballGrabbed)
        {
            currentBall.Rigidbody.position = rightControllerPivot.position;
            if (OVRInput.GetUp(grabButton))
            {
                var localVel = OVRInput.GetLocalControllerVelocity(OVRInput.Controller.RTouch);
                var vel = trackingSpace.TransformVector(localVel);
                var angVel = OVRInput.GetLocalControllerAngularVelocity(OVRInput.Controller.RTouch);
                currentBall.Release(rightControllerPivot.position, vel, angVel);
                ballGrabbed = false;
            }
        }

        if (OVRInput.GetDown(OVRInput.RawButton.RIndexTrigger))
        {
            const float speed = 10f;
            var newBall = Instantiate(ballPrefab).GetComponent<BouncingBallLogic>();
            const float shiftToPreventCollisionWithGrabbedBall = 0.1f;
            var pos = rightControllerPivot.position + rightControllerPivot.forward * shiftToPreventCollisionWithGrabbedBall;
            newBall.Release(pos, rightControllerPivot.forward * speed, Vector3.zero);
        }
    }
}
