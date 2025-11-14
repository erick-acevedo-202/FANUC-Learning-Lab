using UnityEngine;

public class Sonido : MonoBehaviour
{
    public AudioSource audioSource;
    public AudioClip vel1;
    public AudioClip vel2;
    public AudioClip vel3;

    int currentBand = 0; 

    public void StartSoundForVelocity(int velocity)
    {
        int band = 0;

        if (velocity >= 1 && velocity <= 35) band = 1;
        else if (velocity >= 36 && velocity <= 70) band = 2;
        else if (velocity >= 71 && velocity <= 100) band = 3;

        if (band == 0) return;

        
        if (band == currentBand && audioSource.isPlaying) return;

        AudioClip clip = null;
        switch (band)
        {
            case 1: clip = vel1; break;
            case 2: clip = vel2; break;
            case 3: clip = vel3; break;
        }
        if (clip == null) return;

        currentBand = band;
        audioSource.loop = true;
        audioSource.clip = clip;
        audioSource.Play();
    }

    public void StopSound()
    {
        currentBand = 0;
        if (audioSource != null && audioSource.isPlaying)
        {
            audioSource.Stop();
        }
    }
}
