import { Button } from "@mui/material";
import { doc, updateDoc } from "firebase/firestore";
import { db } from "utils/fb";

const InputBtn = ({ number, text }) => {
  return (
    <Button
      variant="contained"
      sx={{
        width: "100%",
        py: "8px",
        fontSize: "25px",
      }}
      onClick={async () => {
        await updateDoc(doc(db, "participantData", "1"), {
          perceivedSafety: number,
        });
      }}
    >
      {number} {text && `(${text})`}
    </Button>
  );
};

export default InputBtn;
