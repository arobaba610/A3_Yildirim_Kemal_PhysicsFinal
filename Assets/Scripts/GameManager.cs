using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class GameManager : MonoBehaviour
{
    [Header("Game Settings")]
    public List<GameObject> birdPrefabs; // List of available bird prefabs
    public Transform birdSpawnPoint; // Where the birds will spawn
    public Transform slingshotAnchor; // Slingshot's anchor point
    public List<GameObject> pigs; // List of all pigs in the level
    public List<GameObject> blocks; // List of all blocks in the level
    public float levelResetDelay = 3f; // Delay before resetting the level after a win or loss

    [Header("UI Settings")]
    public TextMeshProUGUI scoreText; // Displays the score
    public TextMeshProUGUI statusText; // Displays the game status (Win/Lose)
    public Button restartButton; // Button to restart the level
    public Button nextLevelButton; // Button to go to the next level

    private int score = 0; // Player's current score
    private int currentBirdIndex = 0; // Index of the current bird
    private GameObject currentBird; // Reference to the current bird instance
    private bool isLevelComplete = false; // Tracks if the level is complete

    private void Start()
    {
        InitializeGame();
    }

    private void Update()
    {
        if (!isLevelComplete)
        {
            CheckWinCondition();
            CheckLoseCondition();
        }
    }

    private void InitializeGame()
    {
        score = 0;
        currentBirdIndex = 0;
        isLevelComplete = false;

        UpdateScoreUI();
        statusText.text = "";
        restartButton.gameObject.SetActive(false);
        nextLevelButton.gameObject.SetActive(false);

        SpawnBird();
    }

    private void SpawnBird()
    {
        if (currentBirdIndex >= birdPrefabs.Count)
        {
            Debug.Log("No more birds available.");
            return;
        }

        if (birdPrefabs[currentBirdIndex] != null)
        {
            currentBird = Instantiate(birdPrefabs[currentBirdIndex], birdSpawnPoint.position, Quaternion.identity);
            currentBirdIndex++;
        }
        else
        {
            Debug.LogError($"Bird prefab at index {currentBirdIndex} is null!");
        }
    }

    private void CheckWinCondition()
    {
        // Check if all pigs are destroyed
        if (pigs.Count == 0)
        {
            CompleteLevel(true);
        }
    }

    private void CheckLoseCondition()
    {
        // Check if all birds are used and pigs remain
        if (currentBirdIndex >= birdPrefabs.Count && pigs.Count > 0)
        {
            CompleteLevel(false);
        }
    }

    public void OnPigDestroyed(GameObject pig)
    {
        // Update score and remove pig from the list
        score += 100; // Example: 100 points per pig
        pigs.Remove(pig);
        UpdateScoreUI();

        // Check for win condition immediately after a pig is destroyed
        CheckWinCondition();
    }

    public void OnBlockDestroyed(GameObject block)
    {
        // Optionally add points for destroying blocks
        score += 10; // Example: 10 points per block
        blocks.Remove(block);
        UpdateScoreUI();
    }

    public void OnBirdLaunched()
    {
        // Check if there are more birds to spawn
        if (currentBirdIndex < birdPrefabs.Count)
        {
            Invoke(nameof(SpawnBird), 2f); // 
        }
        else
        {
            Debug.Log("No more birds to launch.");
        }
    }

    private void CompleteLevel(bool won)
    {
        isLevelComplete = true;

        if (won)
        {
            statusText.text = "You Win!";
            nextLevelButton.gameObject.SetActive(true);
        }
        else
        {
            statusText.text = "You Lose!";
            restartButton.gameObject.SetActive(true);
        }

       
        Invoke(nameof(ResetLevel), levelResetDelay);
    }

    private void ResetLevel()
    {
    
        UnityEngine.SceneManagement.SceneManager.LoadScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene().buildIndex);
    }

    public void OnNextLevelButtonPressed()
    {
        
        UnityEngine.SceneManagement.SceneManager.LoadScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene().buildIndex + 1);
    }

    public void OnRestartButtonPressed()
    {
      
        ResetLevel();
    }

    private void UpdateScoreUI()
    {
        if (scoreText != null)
        {
            scoreText.text = $"Score: {score}";
        }
    }
}
