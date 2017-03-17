using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections;

public class FramesPerSecond : MonoBehaviour {

	void Start()
	{
		_theText = GetComponent<Text>();
		
		_timeleft = _updateInterval;
	}
	
	void Update()
	{
		_timeleft -= Time.deltaTime;
		_accum += Time.timeScale / Time.deltaTime;
		++_frames;
		
		if(_timeleft <= 0.0)
		{
			var fps = _accum / _frames;
			var format = string.Format("{0:F2} FPS", fps);
			_theText.text = format;
			
			_timeleft = _updateInterval;
			_accum = 0.0f;
			_frames = 0;
		}
	}
	
	private Text _theText;
	private float _updateInterval = 0.5f;
	private float _timeleft;
	private float _accum = 0;
	private int _frames = 0;
}
