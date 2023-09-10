using UnityEngine;
using App.Scripts.Scenes.SceneWordSearch.Features.Level.Models.Level;

namespace App.Scripts.Scenes.SceneWordSearch.Features.Level.BuilderLevelModel.ProviderWordLevel
{
    public class ProviderWordLevel : IProviderWordLevel
    {
        public LevelInfo LoadLevelData(int levelIndex)
        {
            TextAsset jsonLevelFile = Resources.Load<TextAsset>($"WordSearch/Levels/{levelIndex}");

            if (jsonLevelFile)
            {
                return JsonUtility.FromJson<LevelInfo>(jsonLevelFile.text);

            }
            else
            {
                return null;
            }
        }
    }
}