# ROS 2 Deployment with OpenKruise

This section introduces [OpenKruise](https://openkruise.io/) advanced workloads that are useful for ROS 2 application deployment on the edge.

## Reference

- [OpenKruise](https://openkruise.io/) / [GitHub](https://github.com/openkruise/kruise)
- [Installation](https://openkruise.io/docs/installation)
- [In-place Update](https://openkruise.io/docs/core-concepts/inplace-update)
- [CloneSet](https://openkruise.io/docs/user-manuals/cloneset)
- [Advanced DaemonSet](https://openkruise.io/docs/user-manuals/advanceddaemonset)

## Why OpenKruise for Robotics and Edge

Kubernetes native workloads such as `Deployment` and `DaemonSet` always **recreate** the pod to update the application container.
The pod is deleted, re-scheduled, the new pod gets a new name and IP address, the image is pulled if not present, and all containers in the pod are restarted even if only one container image is changed.
This is not a big deal for the stateless web services, but it brings the following problems for ROS 2 applications and robots.

- Downtime: the whole pod is stopped and started, including the sidecar containers that are not changed.
- Re-discovery: ROS 2 / DDS participants need to discover the new pod again since the IP address is changed. Static peers (`ROS_STATIC_PEERS`) and discovery server addresses must be updated.
- Shared resources: `emptyDir` volumes (e.g. `/dev/shm` for [Zero Copy Data Sharing](./ROS2_Deployment_Intermediate.md#ros-2-zero-copy-data-sharing)) are deleted with the pod.
- Edge network: the image pull during the rolling update can be slow or even fail on the edge network, which extends the downtime.

[OpenKruise](https://openkruise.io/) is the CNCF project that extends Kubernetes workloads with the following features.

- [In-place Update](https://openkruise.io/docs/core-concepts/inplace-update): only the changed container is restarted in the **same pod**, the pod name, IP address, sandbox, volumes and the other containers are kept as they are.
- Staged rollout with `partition`: the number (or percentage) of pods that stay on the old revision, this is useful for the canary release to a few robots first.
- [Advanced DaemonSet](https://openkruise.io/docs/user-manuals/advanceddaemonset) with node `selector`: roll out the new version to the specific nodes (robots) first.
- [ImagePullJob](https://openkruise.io/docs/user-manuals/imagepulljob) and image pre-download for in-place update: pull the image to the nodes before the actual update to minimize the downtime.
- [SidecarSet](https://openkruise.io/docs/user-manuals/sidecarset), [ContainerRecreateRequest](https://openkruise.io/docs/user-manuals/containerrecreaterequest) (restart the specific container without recreating the pod), [WorkloadSpread](https://openkruise.io/docs/user-manuals/workloadspread), [PodUnavailableBudget](https://openkruise.io/docs/user-manuals/podunavailablebudget) and so on.

> [!NOTE]
> OpenKruise manages the pods on the standard Kubernetes nodes via `kruise-manager` (controller / webhook) and `kruise-daemon` (DaemonSet on each node for image pulling and container recreation).
> `kruise-daemon` needs to access the Kubernetes API server from the node, so that the KubeEdge edge nodes require [MetaServer](https://kubeedge.io/docs/advanced/metaserver) to run it, which is not covered in this tutorial.

## Install OpenKruise

OpenKruise is installed via Helm chart, see [Installation](https://openkruise.io/docs/installation) for more details and [Kubernetes compatibility](https://openkruise.io/docs/installation#kubernetes-compatibility).
The following procedure is verified with OpenKruise `v1.9.2` on the kind cluster with Kubernetes `v1.37.0`.

```bash
> helm repo add openkruise https://openkruise.github.io/charts/
> helm repo update
> helm install kruise openkruise/kruise --version 1.9.2
NAME: kruise
LAST DEPLOYED: Mon Oct 20 02:45:19 2026
NAMESPACE: default
STATUS: deployed
REVISION: 1
TEST SUITE: None

### kruise-manager and kruise-daemon are deployed in kruise-system namespace
> kubectl get pods -n kruise-system -o wide
NAME                                        READY   STATUS    RESTARTS   AGE   IP             NODE                 NOMINATED NODE   READINESS GATES
kruise-controller-manager-f8ccd48f7-r264j   1/1     Running   0          26s   10.244.1.112   kind-worker          <none>           <none>
kruise-controller-manager-f8ccd48f7-r7h85   1/1     Running   0          26s   10.244.2.230   kind-worker2         <none>           <none>
kruise-daemon-7z7j4                         1/1     Running   0          26s   172.18.0.3     kind-control-plane   <none>           <none>
kruise-daemon-c8rzz                         1/1     Running   0          26s   172.18.0.4     kind-worker          <none>           <none>
kruise-daemon-hf7nt                         1/1     Running   0          26s   172.18.0.2     kind-worker2         <none>           <none>

### Custom Resource Definitions
> kubectl get crd | grep kruise.io
advancedcronjobs.apps.kruise.io              2026-10-19T17:45:30Z
broadcastjobs.apps.kruise.io                 2026-10-19T17:45:30Z
clonesets.apps.kruise.io                     2026-10-19T17:45:30Z
containerrecreaterequests.apps.kruise.io     2026-10-19T17:45:30Z
daemonsets.apps.kruise.io                    2026-10-19T17:45:30Z
imagepulljobs.apps.kruise.io                 2026-10-19T17:45:30Z
sidecarsets.apps.kruise.io                   2026-10-19T17:45:30Z
statefulsets.apps.kruise.io                  2026-10-19T17:45:30Z
...<snip>
```

Optional feature gates can be enabled with `--set featureGates="..."`, e.g. `PreDownloadImageForInPlaceUpdate=true` to pull the new image to the nodes before the in-place update.
Be advised that the pre-download pulls the image from the registry, so that it does not work with the images loaded by `kind load docker-image`.

## CloneSet In-Place Update

[CloneSet](https://openkruise.io/docs/user-manuals/cloneset) is the enhanced `Deployment`, which supports in-place update, partition, `maxSurge`, specified pod deletion and lifecycle hooks.
This example deploys 3 ROS 2 talker pods with `tomoyafujita/ros:jazzy` image, and then updates the container image to `tomoyafujita/ros:rolling` in place.

**see deployment description [ROS 2 CloneSet](./../yaml/ros2-kruise-cloneset.yaml)**

```yaml
  updateStrategy:
    # InPlaceIfPossible: update the container in place (image, metadata) if possible, otherwise recreate the pod.
    type: InPlaceIfPossible
    # Number (or percentage) of pods that stay on the old revision, use it for canary / staged rollout.
    partition: 0
    # Maximum number (or percentage) of pods unavailable during the update.
    maxUnavailable: 1
    inPlaceUpdateStrategy:
      # Delay between the pod is marked as updating (not ready) and the container is actually restarted.
      gracePeriodSeconds: 5
```

- Start CloneSet

  `controller-revision-hash` label tells the revision of the pod template, and `lifecycle.apps.kruise.io/state` label tells the pod state managed by OpenKruise.

```bash
> kubectl apply -f ./yaml/ros2-kruise-cloneset.yaml
cloneset.apps.kruise.io/ros2-talker created

> kubectl get cloneset
NAME          DESIRED   UPDATED   UPDATED_READY   UPDATED_AVAILABLE   READY   TOTAL   AGE
ros2-talker   3         3         3               3                   3       3       20s

> kubectl get pods -l app=ros2-talker -o wide -L controller-revision-hash,lifecycle.apps.kruise.io/state
NAME                READY   STATUS    RESTARTS   AGE   IP             NODE           NOMINATED NODE   READINESS GATES   CONTROLLER-REVISION-HASH   STATE
ros2-talker-2qs6n   1/1     Running   0          20s   10.244.2.108   kind-worker2   <none>           2/2               ros2-talker-cffff5564      Normal
ros2-talker-dctk9   1/1     Running   0          20s   10.244.2.6     kind-worker2   <none>           2/2               ros2-talker-cffff5564      Normal
ros2-talker-rmhx9   1/1     Running   0          20s   10.244.2.133   kind-worker2   <none>           2/2               ros2-talker-cffff5564      Normal

> kubectl exec ros2-talker-2qs6n -- bash -c 'echo $ROS_DISTRO'
jazzy
```

- Canary update with `partition`

  Change the container image to `tomoyafujita/ros:rolling` with `partition: 2`, so that only 1 pod (`replicas - partition`) is updated to the new revision.

> [!IMPORTANT]
> Use JSON patch (`--type json`) or `kubectl edit` / `kubectl apply` to change the image, **do not use** JSON merge patch (`--type merge`) with `containers` list.
> JSON merge patch replaces the whole `containers` list, that drops `command`, `args` and `imagePullPolicy` from the pod template, which is not the in-place updatable change any more and recreates the pods with the default command of the image.

```bash
> kubectl patch cloneset ros2-talker --type json -p '[{"op":"replace","path":"/spec/updateStrategy/partition","value":2},{"op":"replace","path":"/spec/template/spec/containers/0/image","value":"tomoyafujita/ros:rolling"}]'
cloneset.apps.kruise.io/ros2-talker patched

### Only one pod is updating, readiness gate (KruisePodReady) is set to not ready during gracePeriodSeconds
> kubectl get pods -l app=ros2-talker -o wide -L controller-revision-hash,lifecycle.apps.kruise.io/state
NAME                READY   STATUS    RESTARTS   AGE   IP             NODE           NOMINATED NODE   READINESS GATES   CONTROLLER-REVISION-HASH   STATE
ros2-talker-2qs6n   1/1     Running   0          25s   10.244.2.108   kind-worker2   <none>           2/2               ros2-talker-cffff5564      Normal
ros2-talker-dctk9   1/1     Running   0          25s   10.244.2.6     kind-worker2   <none>           1/2               ros2-talker-5f7494c5cf     Updating
ros2-talker-rmhx9   1/1     Running   0          25s   10.244.2.133   kind-worker2   <none>           2/2               ros2-talker-cffff5564      Normal

> kubectl get cloneset
NAME          DESIRED   UPDATED   UPDATED_READY   UPDATED_AVAILABLE   READY   TOTAL   AGE
ros2-talker   3         1         0                                   2       3       50s
```

- Roll out to all pods

  Set `partition: 0` to update the rest of pods.
  As you can see below, **the pod names and IP addresses are not changed**, only the container is restarted (`RESTARTS` is 1) with the new image.

```bash
> kubectl patch cloneset ros2-talker --type json -p '[{"op":"replace","path":"/spec/updateStrategy/partition","value":0}]'
cloneset.apps.kruise.io/ros2-talker patched

> kubectl get pods -l app=ros2-talker -o wide -L controller-revision-hash,lifecycle.apps.kruise.io/state
NAME                READY   STATUS    RESTARTS      AGE    IP             NODE           NOMINATED NODE   READINESS GATES   CONTROLLER-REVISION-HASH   STATE
ros2-talker-2qs6n   1/1     Running   1 (10s ago)   100s   10.244.2.108   kind-worker2   <none>           2/2               ros2-talker-5f7494c5cf     Normal
ros2-talker-dctk9   1/1     Running   1 (45s ago)   100s   10.244.2.6     kind-worker2   <none>           2/2               ros2-talker-5f7494c5cf     Normal
ros2-talker-rmhx9   1/1     Running   0             100s   10.244.2.133   kind-worker2   <none>           1/2               ros2-talker-5f7494c5cf     Updating

> kubectl get cloneset
NAME          DESIRED   UPDATED   UPDATED_READY   UPDATED_AVAILABLE   READY   TOTAL   AGE
ros2-talker   3         3         3               3                   3       3       2m

> kubectl exec ros2-talker-2qs6n -- bash -c 'echo $ROS_DISTRO'
rolling
> kubectl logs ros2-talker-2qs6n --tail=1
[INFO] [1792432521.004275838] [talker]: Publishing: 'Hello World: 9'

### kubelet restarts the container in the same pod sandbox
> kubectl get events --field-selector involvedObject.name=ros2-talker-2qs6n --sort-by=.lastTimestamp
LAST SEEN   TYPE     REASON      OBJECT                  MESSAGE
100s        Normal   Scheduled   pod/ros2-talker-2qs6n   Successfully assigned default/ros2-talker-2qs6n to kind-worker2
100s        Normal   Pulled      pod/ros2-talker-2qs6n   Container image "tomoyafujita/ros:jazzy" already present on machine and can be accessed by the pod
40s         Normal   Killing     pod/ros2-talker-2qs6n   Container ros2-talker definition changed, will be restarted
10s         Normal   Pulled      pod/ros2-talker-2qs6n   Container image "tomoyafujita/ros:rolling" already present on machine and can be accessed by the pod
10s         Normal   Created     pod/ros2-talker-2qs6n   Container created
10s         Normal   Started     pod/ros2-talker-2qs6n   Container started

### In-place update state is recorded in the pod annotation
> kubectl get pod ros2-talker-2qs6n -o jsonpath='{.metadata.annotations.apps\.kruise\.io/inplace-update-state}'
{"revision":"ros2-talker-5f7494c5cf","updateTimestamp":"2026-10-19T17:54:36Z","lastContainerStatuses":{"ros2-talker":{"imageID":"..."}},"updateImages":true,"containerBatchesRecord":[{"timestamp":"2026-10-19T17:54:41Z","containers":["ros2-talker"]}]}
```

- Stop CloneSet

```bash
> kubectl delete -f ./yaml/ros2-kruise-cloneset.yaml
cloneset.apps.kruise.io "ros2-talker" deleted
```

## Advanced DaemonSet Rollout by Node Selector

[Advanced DaemonSet](https://openkruise.io/docs/user-manuals/advanceddaemonset) is the enhanced `DaemonSet` with `apps.kruise.io/v1beta1` API version, which supports in-place update, `partition`, `paused` and `selector` based rolling update.
With the node `selector`, the new revision is rolled out only to the nodes that match the label, this is useful to update the specific robots first and then the whole fleet.

**see deployment description [ROS 2 Advanced DaemonSet](./../yaml/ros2-kruise-daemonset.yaml)**

```yaml
  updateStrategy:
    type: RollingUpdate
    rollingUpdate:
      # InPlaceIfPossible: update the container in place on the node, Standard: recreate the pod (the same with DaemonSet).
      rollingUpdateType: InPlaceIfPossible
      maxUnavailable: 1
      # Only the pods on the nodes that match the selector are updated, the others stay on the old revision.
      selector:
        matchLabels:
          ros2-rollout: canary
      partition: 0
      paused: false
```

- Start Advanced DaemonSet

```bash
> kubectl apply -f ./yaml/ros2-kruise-daemonset.yaml
daemonset.apps.kruise.io/ros2-listener created

> kubectl get daemonsets.apps.kruise.io
NAME            DESIRED   CURRENT   READY   UP-TO-DATE   AVAILABLE   AGE
ros2-listener   3         3         3       3            3           20s

> kubectl get pods -l app=ros2-listener -o wide -L controller-revision-hash
NAME                  READY   STATUS    RESTARTS   AGE   IP             NODE                 NOMINATED NODE   READINESS GATES   CONTROLLER-REVISION-HASH
ros2-listener-69skh   1/1     Running   0          20s   10.244.2.67    kind-worker2         <none>           2/2               7755c89657
ros2-listener-9gdd5   1/1     Running   0          20s   10.244.0.124   kind-control-plane   <none>           2/2               7755c89657
ros2-listener-lf9x7   1/1     Running   0          20s   10.244.1.223   kind-worker          <none>           2/2               7755c89657
```

- Canary update to the selected node

  Label the node to roll out the new revision first, and change the container image.
  Only the pod on `kind-worker` is updated in place, the others stay on the old revision.

```bash
> kubectl label nodes kind-worker ros2-rollout=canary
> kubectl patch daemonsets.apps.kruise.io ros2-listener --type json -p '[{"op":"replace","path":"/spec/template/spec/containers/0/image","value":"tomoyafujita/ros:rolling"}]'
daemonset.apps.kruise.io/ros2-listener patched

> kubectl get daemonsets.apps.kruise.io
NAME            DESIRED   CURRENT   READY   UP-TO-DATE   AVAILABLE   AGE
ros2-listener   3         3         3       1            3           55s

> kubectl get pods -l app=ros2-listener -o wide -L controller-revision-hash
NAME                  READY   STATUS    RESTARTS     AGE   IP             NODE                 NOMINATED NODE   READINESS GATES   CONTROLLER-REVISION-HASH
ros2-listener-69skh   1/1     Running   0            55s   10.244.2.67    kind-worker2         <none>           2/2               7755c89657
ros2-listener-9gdd5   1/1     Running   0            55s   10.244.0.124   kind-control-plane   <none>           2/2               7755c89657
ros2-listener-lf9x7   1/1     Running   1 (5s ago)   55s   10.244.1.223   kind-worker          <none>           2/2               848ddf7c5b

> for p in $(kubectl get pods -l app=ros2-listener -o jsonpath='{.items[*].metadata.name}'); do echo -n "$p: "; kubectl exec $p -- bash -c 'echo $ROS_DISTRO'; done
ros2-listener-69skh: jazzy
ros2-listener-9gdd5: jazzy
ros2-listener-lf9x7: rolling
```

- Roll out to all nodes

  Remove the `selector` (or label all nodes) to update the rest of nodes.

```bash
> kubectl patch daemonsets.apps.kruise.io ros2-listener --type json -p '[{"op":"remove","path":"/spec/updateStrategy/rollingUpdate/selector"}]'
daemonset.apps.kruise.io/ros2-listener patched

> kubectl get pods -l app=ros2-listener -o wide -L controller-revision-hash
NAME                  READY   STATUS    RESTARTS      AGE    IP             NODE                 NOMINATED NODE   READINESS GATES   CONTROLLER-REVISION-HASH
ros2-listener-69skh   1/1     Running   1 (20s ago)   105s   10.244.2.67    kind-worker2         <none>           2/2               848ddf7c5b
ros2-listener-9gdd5   1/1     Running   1 (3s ago)    105s   10.244.0.124   kind-control-plane   <none>           2/2               848ddf7c5b
ros2-listener-lf9x7   1/1     Running   1 (55s ago)   105s   10.244.1.223   kind-worker          <none>           2/2               848ddf7c5b
```

- Stop Advanced DaemonSet

```bash
> kubectl delete -f ./yaml/ros2-kruise-daemonset.yaml
daemonset.apps.kruise.io "ros2-listener" deleted
> kubectl label nodes kind-worker ros2-rollout-
```

## Uninstall OpenKruise

Delete all OpenKruise resources (CloneSet, Advanced DaemonSet and so on) before uninstalling, see [Uninstall](https://openkruise.io/docs/installation#uninstall).

```bash
> helm uninstall kruise
```
