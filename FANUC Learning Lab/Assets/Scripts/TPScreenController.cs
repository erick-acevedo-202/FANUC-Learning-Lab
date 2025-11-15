using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TPScreenController : MonoBehaviour
{


    [Header("Status Indicators Highlights")]
    public Image BusyOff;
    public Image RunOff;
    public Image StepOff;
    public Image IOOff;
    public Image HoldOff;
    public Image ProdOff;
    public Image FaultOff;
    public Image TcycOff;


    void Start()
    {
        statusRunningOK();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void turnOffIndicator(Image statusIndicator)
    {
        statusIndicator.color = new Color32(30, 40, 50, 200);
    }

    private void turnOnIndicator(Image statusIndicator)
    {
        statusIndicator.color = new Color32(30, 40, 50, 0);
    }

    private void turnOffAll()
    {
        turnOffIndicator(BusyOff);
        turnOffIndicator(RunOff);
        turnOffIndicator(StepOff);
        turnOffIndicator(IOOff);
        turnOffIndicator(HoldOff);
        turnOffIndicator(ProdOff);
        turnOffIndicator(FaultOff);
        turnOffIndicator(TcycOff);
    }

    public void statusRunningOK()
    {
        turnOffAll();
        turnOnIndicator(BusyOff);
        turnOnIndicator(RunOff);
        turnOnIndicator(StepOff);
        turnOnIndicator(ProdOff);
    }

    public void statusFailed() {
        turnOnIndicator(FaultOff);
    }

}
