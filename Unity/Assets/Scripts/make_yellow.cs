using UnityEngine;

public class ChangeColor : MonoBehaviour
{
    void Start()
    {
        // �� ��ũ��Ʈ�� ������ ������Ʈ�� Material ������Ʈ�� �����Ͽ� ������ ��������� ����
        GetComponent<Renderer>().material.color = Color.yellow;
    }
}
