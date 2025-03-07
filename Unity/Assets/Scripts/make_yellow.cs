using UnityEngine;

public class ChangeColor : MonoBehaviour
{
    void Start()
    {
        // 이 스크립트가 부착된 오브젝트의 Material 컴포넌트에 접근하여 색상을 노란색으로 변경
        GetComponent<Renderer>().material.color = Color.yellow;
    }
}
