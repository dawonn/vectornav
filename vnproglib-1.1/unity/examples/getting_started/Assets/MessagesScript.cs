using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class MessagesScript : MonoBehaviour {
	
	void Start()
	{
		_theText = GetComponent<Text>();
		_theText.text = string.Empty;
	}

	void Update()
	{
	
	}

	private Text _theText;
}
