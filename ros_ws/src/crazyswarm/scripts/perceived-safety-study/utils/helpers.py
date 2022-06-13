def userInput(text, inputType=str, validAnswers=set()):
    valid = False

    while not valid:
        try:
            textInput = input(text)

            if textInput == "q":
                valid = True
            else:
                textInput = inputType(textInput)

                if textInput not in validAnswers and len(validAnswers) > 0:
                    print("Answer not in list of approved answer")
                else:
                    valid = True
        except:
            print("Invalid input")


    if textInput == "q":
        exit()
    else:
        return textInput