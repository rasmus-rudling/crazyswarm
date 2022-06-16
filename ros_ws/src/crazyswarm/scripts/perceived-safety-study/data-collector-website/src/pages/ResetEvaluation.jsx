import { Box, Typography } from "@mui/material";
import InputBtn from "components/InputBtn";

import { doc, onSnapshot } from "firebase/firestore";
import { useEffect, useState } from "react";
import { db } from "utils/fb";

const ResetEvaluation = () => {
  const [perceivedSafety, setPerceivedSafety] = useState(99);

  useEffect(() => {
    return onSnapshot(doc(db, "participantData", "1"), (doc) => {
      setPerceivedSafety(doc.data().perceivedSafety);
    });
  }, []);

  return (
    <div>
      <Box
        sx={{
          display: "flex",
          alignItems: "center",
          flexDirection: "column",
          justifyContent: "center",
          pt: "10px",
          rowGap: "20px",
          flexWrap: "wrap",
          width: "100%",
          backgroundColor: perceivedSafety === 99 ? "white" : "red",
          position: "absolute",
          top: "0px",
          right: "0px",
          bottom: "0px",
          left: "0px",
        }}
      >
        <Typography
          variant="h4"
          sx={{
            textAlign: "center",
            fontSize: "30px",
            color: "white",
            px: "10px",
          }}
        >
          Currently selected perceived safety:
        </Typography>

        <Typography
          variant="h4"
          sx={{
            textAlign: "center",
            fontSize: "200px",
            color: "white",
          }}
        >
          {perceivedSafety}
        </Typography>

        {perceivedSafety !== 99 && (
          <InputBtn number={99} text="Reset" sx={{ py: "30px" }} />
        )}
      </Box>
    </div>
  );
};

export default ResetEvaluation;
