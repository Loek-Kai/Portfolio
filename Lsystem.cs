using System;
using System.Text;
using System.Collections.Generic;
using UnityEngine;

public class TransformInfo
{
  public Vector3 position;
  public Quaternion rotation;
}

public class Lsystem : MonoBehaviour
{
    //Variables for amount of generating
    [SerializeField] private int iterations = 4;
    [SerializeField] private float length = 10f;
    [SerializeField] private float angle = 30f;
    [SerializeField] private GameObject branch;

    //Strings for generating
    private string currentString = string.Empty;
    private const string axiom = "X";

    //Information for generating
    private Stack<TransformInfo> transformStack;
    private Dictionary<char, string> rules;
    

    void Start()
    {
        transformStack = new Stack<TransformInfo>();
    }

    public void GenerateTree()
    {
        rules = new Dictionary<char, string>
        {
            {'X', "-F[+F][--X]+F-F[++++X]-X" },
            {'F', "FF" }
        };

        Generate();
    }

}
