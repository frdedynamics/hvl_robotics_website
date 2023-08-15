document.addEventListener("DOMContentLoaded", function() {
    var toggleButton = document.getElementById("toggleButton");
    var hiddenText = document.getElementById("hiddenText");
  
    toggleButton.addEventListener("click", function() {
      if (hiddenText.style.display === "none") {
        hiddenText.style.display = "block";
      } else {
        hiddenText.style.display = "none";
      }
    });
  });
  