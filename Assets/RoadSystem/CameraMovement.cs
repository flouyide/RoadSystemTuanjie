using UnityEngine;
using UnityEngine.InputSystem;

public class CameraMovement : MonoBehaviour
{
    [Header("Movement")]
    [SerializeField] private float moveSpeed = 20f;
    [SerializeField] private float scrollSpeed = 10f;
    [SerializeField] private float minHeight = 5f;
    [SerializeField] private float maxHeight = 100f;

    [Header("Rotation")]
    [SerializeField] private float rotateSpeed = 90f;

    [Header("Input Actions")]
    [SerializeField] private InputActionReference moveAction;   // Vector2，绑定 WASD
    [SerializeField] private InputActionReference scrollAction; // Vector2，绑定 Mouse/Scroll
    [SerializeField] private InputActionReference rotateAction; // float，Q=+1 / E=-1

    private void OnEnable()
    {
        if (moveAction != null) moveAction.action.Enable();
        if (scrollAction != null) scrollAction.action.Enable();
        if (rotateAction != null) rotateAction.action.Enable();
    }

    private void OnDisable()
    {
        if (moveAction != null) moveAction.action.Disable();
        if (scrollAction != null) scrollAction.action.Disable();
        if (rotateAction != null) rotateAction.action.Disable();
    }

    private void Update()
    {
        // QE 旋转（绕 Y 轴）
        if (rotateAction != null && rotateAction.action.enabled)
        {
            float rotate = rotateAction.action.ReadValue<float>();
            transform.Rotate(0f, rotate * rotateSpeed * Time.deltaTime, 0f, Space.World);
        }

        // WASD 移动（以镜头在 XZ 平面的投影朝向为前进方向）
        if (moveAction != null && moveAction.action.enabled)
        {
            Vector2 move = moveAction.action.ReadValue<Vector2>();
            Vector3 forward = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
            Vector3 right   = Vector3.ProjectOnPlane(transform.right,   Vector3.up).normalized;
            Vector3 delta   = (forward * move.y + right * move.x) * moveSpeed * Time.deltaTime;
            transform.Translate(delta, Space.World);
        }

        // 滚轮控制高度（Y 轴）
        if (scrollAction != null && scrollAction.action.enabled)
        {
            Vector2 scroll = scrollAction.action.ReadValue<Vector2>();
            float newY = Mathf.Clamp(
                transform.position.y - scroll.y * scrollSpeed * Time.deltaTime,
                minHeight, maxHeight);
            transform.position = new Vector3(transform.position.x, newY, transform.position.z);
        }
    }
}
